#include "vln_agent/session/session_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "vln_agent/session/responses_ws_transport.hpp"

namespace vln_agent
{
namespace
{
using Clock = std::chrono::steady_clock;

/** @brief 会话固定配置；仅 fields 中的字段进入模型请求。 */
struct SessionConfig
{
  std::string base_url, api_key;
  Json fields;
  Clock::duration interval, timeout;
  bool stream_output;
  std::uint64_t budget, threshold;
};

/** @brief 读取独立配置段，不解释工具、提示词文件或业务输入。 */
SessionConfig readConfig(const std::string & path)
{
  const auto yaml = YAML::LoadFile(path)["session_manager"];
  if (!yaml.IsMap()) {
    throw std::invalid_argument("缺少 session_manager 配置");
  }
  SessionConfig config;
  config.base_url = yaml["base_url"].as<std::string>();
  config.api_key = yaml["api_key"].as<std::string>();
  const auto hz = yaml["max_request_hz"].as<double>(1.0);
  const auto timeout = yaml["request_timeout_sec"].as<double>(0.0);
  const auto context = yaml["context"];
  config.budget = context ? context["budget_tokens"].as<std::uint64_t>(128000) : 128000;
  const auto ratio = context ? context["compact_ratio"].as<double>(0.8) : 0.8;
  const auto max_output = yaml["max_output_tokens"].as<int>(8192);
  if (!std::isfinite(hz) || hz <= 0 || !std::isfinite(timeout) || timeout < 0 ||
    config.budget == 0 || !std::isfinite(ratio) || ratio <= 0 || ratio >= 1 || max_output <= 0)
  {
    throw std::invalid_argument("会话频率、超时、输出上限或上下文预算不合法");
  }
  config.threshold = static_cast<std::uint64_t>(std::floor(config.budget * ratio));
  if (config.threshold == 0) {
    throw std::invalid_argument("压缩阈值必须大于零");
  }
  config.interval = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1 / hz));
  config.timeout = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
  config.stream_output = yaml["stream_output"].as<bool>(false);
  config.fields = {
    {"type", "response.create"}, {"model", yaml["model"].as<std::string>()},
    {"store", yaml["store"].as<bool>(true)},
    {"reasoning", {{"effort", yaml["reasoning_effort"].as<std::string>("low")}}},
    {"max_output_tokens", max_output},
    {"parallel_tool_calls", yaml["parallel_tool_calls"].as<bool>(true)},
    {"prompt_cache_key", yaml["prompt_cache_key"].as<std::string>("scout-mini-vln")},
    {"text", {{"format", {{"type", "text"}}}}},
    {"context_management", Json::array({
          {{"type", "compaction"}, {"compact_threshold", config.threshold}}})}};
  return config;
}
}  // namespace

/** @brief 队列与网络互相独立；会话锁不跨越 send、interrupt、close 或 join。 */
struct SessionManager::Impl
{
  explicit Impl(const std::string & path) : config(readConfig(path))
  {
    if (config.stream_output) {
      stream = {{"response_id", ""}, {"state", "idle"}, {"content", Json::array()}};
    }
  }

  ~Impl() {shutdown();}

  void start()
  {
    transport.connect(config.base_url, config.api_key,
      [this](const Json & event) {onEvent(event);},
      [this](const Json & state) {onState(state);});
    scheduler = std::thread([this] {schedule();});
  }

  void ensureOpen() const
  {
    if (closed) {
      throw std::logic_error("SessionManager 已经关闭");
    }
  }

  /** @brief 唯一请求构建入口，发送时才读取工具和最新完成链头。 */
  Json buildRequest()
  {
    Json request = config.fields;
    request["tools"] = tools;
    request["input"] = std::move(pending_input);
    pending_input = Json::array();
    if (!latest_response_id.empty()) {
      request["previous_response_id"] = latest_response_id;
    }
    return request;
  }

  /** @brief 在会话锁内记录会话错误，保留当前与待发输入。 */
  void fail(const std::string & code, const std::string & message, const Json & event = nullptr)
  {
    last_error = {{"code", code}, {"message", message}, {"event", event}};
    context_invalid = code == "previous_response_not_found";
    in_flight = false;
    last_finished = Clock::now();
    send_requested = false;
    compaction_items.clear();
    if (config.stream_output) {
      stream["state"] = "error";
    }
  }

  /** @brief 通道失效时中止当前轮，重连成功只恢复可用性，不登记重发意图。 */
  void onState(const Json & state)
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (closed) {
        return;
      }
      available = state.at("connected").get<bool>();
      if (!available && in_flight) {
        in_flight = false;
        last_finished = Clock::now();
        send_requested = false;
        if (config.stream_output) {
          stream["state"] = "interrupted";
        }
      }
    }
    condition.notify_all();
  }

  /** @brief 按本轮项目标志去重压缩记录，保留原始加密项而不解读内容。 */
  void observeCompaction(const Json & item, std::size_t index)
  {
    if (item.value("type", std::string()) == "compaction") {
      const auto key = item.value("id", "index_" + std::to_string(index));
      if (compaction_items.insert(key).second) {
        ++compaction_count;
      }
    }
  }

  /** @brief 仅更新显示文本，不拼接工具参数或重建业务结果。 */
  void appendText(const Json & event)
  {
    const auto output_index = event.at("output_index").get<std::size_t>();
    const auto content_index = event.at("content_index").get<std::size_t>();
    const auto type = event.at("type") == "response.refusal.delta" ? "refusal" : "output_text";
    for (auto & part : stream["content"]) {
      if (part.at("output_index") == output_index && part.at("content_index") == content_index) {
        part["text"].get_ref<std::string &>() += event.at("delta").get<std::string>();
        return;
      }
    }
    stream["content"].push_back({
        {"output_index", output_index}, {"content_index", content_index},
        {"type", type}, {"text", event.at("delta")}});
  }

  /** @brief 完成后用完整文本校准显示快照，完整 response 本身仍原样返回。 */
  void finishText(const Json & response)
  {
    stream["state"] = "completed";
    stream["response_id"] = response.at("id");
    stream["content"] = Json::array();
    const auto & output = response.at("output");
    for (std::size_t i = 0; i < output.size(); ++i) {
      if (output[i].value("type", std::string()) != "message") {
        continue;
      }
      const auto & content = output[i].at("content");
      for (std::size_t j = 0; j < content.size(); ++j) {
        const auto type = content[j].value("type", std::string());
        if (type == "output_text" || type == "refusal") {
          stream["content"].push_back({
              {"output_index", i}, {"content_index", j}, {"type", type},
              {"text", content[j].at(type == "refusal" ? "refusal" : "text")}});
        }
      }
    }
  }

  /** @brief 处理正常连接上的模型事件；网络异常不会通过此入口伪造。 */
  void onEvent(const Json & event)
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (closed || !in_flight) {
        return;
      }
      try {
        const auto type = event.at("type").get<std::string>();
        if (type == "response.created" || type == "response.in_progress") {
          active_response_id = event.at("response").at("id").get<std::string>();
          if (config.stream_output) {
            stream["response_id"] = active_response_id;
          }
        } else if (type == "response.output_text.delta" || type == "response.refusal.delta") {
          if (config.stream_output) {
            appendText(event);
          }
        } else if (type == "response.output_item.done") {
          observeCompaction(event.at("item"), event.at("output_index").get<std::size_t>());
        } else if (type == "response.completed") {
          const auto & response = event.at("response");
          const auto id = response.at("id").get<std::string>();
          if (!active_response_id.empty() && id != active_response_id) {
            return;
          }
          if (id.empty() || response.at("status") != "completed" || !response.at("output").is_array()) {
            throw std::invalid_argument("完成事件缺少完整 response");
          }
          const auto & output = response.at("output");
          for (std::size_t i = 0; i < output.size(); ++i) {
            observeCompaction(output[i], i);
          }
          if (config.stream_output) {
            finishText(response);
          }
          latest_response_id = id;
          active_response_id = id;
          last_finished = Clock::now();
          usage = response.value("usage", Json());
          response_slot = response;
          compaction_items.clear();
          current_request = nullptr;
          in_flight = false;
        } else if (type == "error" || type == "response.failed" || type == "response.incomplete") {
          const auto & source = type == "error" ? event : event.at("response");
          const auto error = source.value("error", Json());
          const auto code = error.is_object() ?
            error.value("code", type) : type;
          const auto message = error.is_object() ?
            error.value("message", std::string("模型请求失败")) : std::string("模型请求未完整完成");
          fail(code, message, event);
        }
      } catch (const std::exception & error) {
        fail("INVALID_RESPONSE", error.what(), event);
      }
    }
    condition.notify_all();
  }

  /** @brief 结果未取走时不覆盖；限频和超时均使用 steady_clock 的定时唤醒。 */
  void schedule()
  {
    std::unique_lock<std::mutex> lock(mutex);
    while (!closed) {
      const auto now = Clock::now();
      if (in_flight && config.timeout != Clock::duration::zero() &&
        now >= last_started + config.timeout)
      {
        fail("REQUEST_TIMEOUT", "等待模型响应超时");
        // 在换连接完成前关闭发送入口，隔离旧轮的迟到事件。
        available = false;
        lock.unlock();
        transport.interrupt();
        lock.lock();
        continue;
      }
      const bool eligible = send_requested && !pending_input.empty() &&
        !in_flight && response_slot.is_null() && last_error.is_null() && available;
      const auto allowed_at = ever_started ? last_started + config.interval : now;
      if (eligible && now >= allowed_at) {
        current_request = buildRequest();
        send_requested = false;
        in_flight = true;
        last_started = now;
        last_finished = {};
        ever_started = true;
        active_response_id.clear();
        compaction_items.clear();
        if (config.stream_output) {
          stream = {{"response_id", ""}, {"state", "generating"}, {"content", Json::array()}};
        }
        auto frame = current_request;
        lock.unlock();
        transport.send(std::move(frame));
        lock.lock();
        continue;
      }
      auto wake = Clock::time_point::max();
      if (in_flight && config.timeout != Clock::duration::zero()) {
        wake = last_started + config.timeout;
      }
      if (eligible) {
        wake = std::min(wake, allowed_at);
      }
      if (wake == Clock::time_point::max()) {
        condition.wait(lock);
      } else {
        condition.wait_until(lock, wake);
      }
    }
  }

  std::string state() const
  {
    if (closed) {return "closed";}
    if (!last_error.is_null()) {return "error";}
    if (!response_slot.is_null()) {return "response_ready";}
    if (!available) {return "waiting_transport";}
    if (in_flight) {return "generating";}
    if (send_requested && ever_started && Clock::now() < last_started + config.interval) {
      return "waiting_interval";
    }
    return "idle";
  }

  void shutdown()
  {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex);
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (closed) {
        return;
      }
      closed = true;
      if (in_flight) {
        last_finished = Clock::now();
      }
      send_requested = false;
    }
    condition.notify_all();
    transport.close();
    if (scheduler.joinable()) {
      scheduler.join();
    }
    std::lock_guard<std::mutex> lock(mutex);
    current_request = nullptr;
    pending_input = Json::array();
    response_slot = nullptr;
    in_flight = false;
    last_error = nullptr;
    usage = nullptr;
    latest_response_id.clear();
    active_response_id.clear();
    compaction_items.clear();
    if (config.stream_output) {
      stream = {{"response_id", ""}, {"state", "closed"}, {"content", Json::array()}};
    }
  }

  SessionConfig config;                    ///< 不变的模型配置及限频策略。
  mutable std::mutex mutex;                ///< 保护缓冲、上下文和事件状态。
  std::mutex lifecycle_mutex;              ///< 串行化关闭和线程回收。
  std::condition_variable condition;
  Json tools = Json::array();              ///< 下次发送时复制到请求。
  Json current_request;                    ///< 当前已发送快照，失败后保留用于显式恢复。
  Json pending_input = Json::array();       ///< 持续追加的下一轮输入，不是请求队列。
  Json response_slot;                      ///< 唯一完整结果槽。
  Json last_error, usage, stream;           ///< 会话错误、最近用量和可选显示。
  std::string latest_response_id, active_response_id;
  std::set<std::string> compaction_items;   ///< 本轮已观察到的压缩项，避免流式/终态双计数。
  std::uint64_t compaction_count{0};
  bool closed{false}, available{false}, in_flight{false}, send_requested{false};
  bool ever_started{false}, context_invalid{false};
  Clock::time_point last_started{}, last_finished{};  ///< 本轮起止时刻，用于稳定的耗时查询。
  ResponsesWsTransport transport;          ///< 网络状态机完全留在传输层。
  std::thread scheduler;
};

SessionManager::SessionManager(const std::string & global_yaml_path)
: impl_(std::make_unique<Impl>(global_yaml_path))
{
  impl_->start();
}

SessionManager::~SessionManager() = default;

void SessionManager::setTools(const Json & tools)
{
  if (!tools.is_array()) {
    throw std::invalid_argument("tools 必须为数组");
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->ensureOpen();
  impl_->tools = tools;
}

Json SessionManager::getTools() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->tools;
}

Json SessionManager::getPendingInputs() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->pending_input;
}

Json SessionManager::getCurrentRequest() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->current_request;
}

void SessionManager::addInputs(const Json & input)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->ensureOpen();
  utils::appendInputs(impl_->pending_input, input);
}

void SessionManager::addInputs(const Json::array_t & input_items)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->ensureOpen();
  utils::appendInputs(impl_->pending_input, input_items);
}

void SessionManager::request()
{
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->ensureOpen();
    if (impl_->context_invalid) {
      return;
    }
    if (!impl_->in_flight && !impl_->current_request.is_null()) {
      utils::prependArray(impl_->pending_input, impl_->current_request.at("input"));
      impl_->current_request = nullptr;
      impl_->last_error = nullptr;
    }
    if (!impl_->pending_input.empty()) {
      impl_->send_requested = true;
    }
  }
  impl_->condition.notify_all();
}

Json SessionManager::takeResponse()
{
  Json result;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    result = std::move(impl_->response_slot);
    impl_->response_slot = nullptr;
    if (!result.is_null() && impl_->config.stream_output) {
      impl_->stream = {{"response_id", ""}, {"state", "idle"}, {"content", Json::array()}};
    }
  }
  impl_->condition.notify_all();
  return result;
}

Json SessionManager::getRequestStatus() const
{
  Json status;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto now = Clock::now();
    const auto ended = impl_->last_finished == Clock::time_point{} ? now : impl_->last_finished;
    const auto elapsed = impl_->ever_started ?
      std::chrono::duration_cast<std::chrono::milliseconds>(ended - impl_->last_started).count() : 0;
    const auto rate_wait = impl_->ever_started ?
      std::max<std::int64_t>(0, std::chrono::ceil<std::chrono::milliseconds>(
        impl_->last_started + impl_->config.interval - now).count()) : 0;
    const bool retry_required =
      !impl_->closed && !impl_->in_flight && !impl_->current_request.is_null();
    status = {
      {"current_response_id", impl_->active_response_id},
      {"retry_required", retry_required},
      {"can_retry", retry_required && !impl_->context_invalid},
      {"request_elapsed_ms", elapsed}, {"rate_limit_remaining_ms", rate_wait},
      {"state", impl_->state()}, {"send_requested", impl_->send_requested},
      {"pending_input_count", impl_->pending_input.size()},
      {"current_input_count", impl_->current_request.is_null() ?
          0 : impl_->current_request.at("input").size()},
      {"in_flight", impl_->in_flight}, {"response_ready", !impl_->response_slot.is_null()},
      {"request_interrupted", !impl_->in_flight && !impl_->current_request.is_null()},
      {"last_error", impl_->last_error}};
  }
  status["transport"] = impl_->transport.getStatus();
  return status;
}

Json SessionManager::getContextStatus() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return {
    {"latest_response_id", impl_->latest_response_id},
    {"budget_tokens", impl_->config.budget}, {"compact_threshold", impl_->config.threshold},
    {"usage", impl_->usage}, {"compaction_count", impl_->compaction_count}};
}

Json SessionManager::getStreamSnapshot() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->stream;
}

void SessionManager::shutdown() {impl_->shutdown();}

}  // namespace vln_agent
