#include "vln_agent/session/session_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <yaml-cpp/yaml.h>

#include "vln_agent/session/responses_transport.hpp"
#include "local_context.hpp"

namespace vln_agent
{
namespace
{
using Clock = std::chrono::steady_clock;

/** @brief 会话固定配置；仅 fields 中的字段进入模型请求。 */
struct SessionConfig
{
  std::string base_url, api_key, transport;
  Json fields;
  Clock::duration interval, timeout;
  bool stream_output, local_history, forward_tool_async;
  std::uint64_t budget, threshold, recent_items, summary_tokens;
};

/** @brief 读取独立配置段，不解释工具、提示词文件或业务输入。 */
SessionConfig readConfig(const std::string & path)
{
  const auto yaml = YAML::LoadFile(path)["session_manager"];
  SessionConfig config;
  config.base_url = yaml["base_url"].as<std::string>();
  config.api_key = yaml["api_key"].as<std::string>();
  const auto hz = yaml["max_request_hz"].as<double>(1.0);
  const auto timeout = yaml["request_timeout_sec"].as<double>(0.0);
  config.transport = yaml["transport_plugin"].as<std::string>("vln_agent/ResponsesWsTransport");
  const auto context = yaml["context"];
  const auto mode = context ? context["mode"].as<std::string>("server") : "server";
  if (mode != "server" && mode != "local") {
    throw std::invalid_argument("context.mode 必须为 server 或 local");
  }
  config.local_history = mode == "local";
  config.recent_items = context ? context["keep_recent_items"].as<std::uint64_t>(8) : 8;
  config.summary_tokens = context ? context["summary_max_output_tokens"].as<std::uint64_t>(1500) : 1500;
  config.budget = context ? context["budget_tokens"].as<std::uint64_t>(128000) : 128000;
  const auto ratio = context ? context["compact_ratio"].as<double>(0.8) : 0.8;
  const auto max_output = yaml["max_output_tokens"].as<int>(8192);
  if (!std::isfinite(hz) || hz <= 0 || !std::isfinite(timeout) || timeout < 0 ||
    config.budget == 0 || !std::isfinite(ratio) || ratio <= 0 || ratio >= 1 || max_output <= 0)
  {
    throw std::invalid_argument("会话频率、超时、输出上限或上下文预算不合法");
  }
  if (config.local_history &&
    (static_cast<std::uint64_t>(max_output) >= config.budget ||
    config.summary_tokens == 0 || config.summary_tokens >= config.budget))
  {
    throw std::invalid_argument("本地会话必须为输出和摘要预留预算");
  }
  const auto input_budget = config.local_history ? config.budget - max_output : config.budget;
  config.threshold = static_cast<std::uint64_t>(std::floor(input_budget * ratio));
  if (config.threshold == 0) {
    throw std::invalid_argument("压缩阈值必须大于零");
  }
  config.interval = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1 / hz));
  config.timeout = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
  config.stream_output = yaml["stream_output"].as<bool>(false);
  config.forward_tool_async = yaml["forward_tool_async"].as<bool>(false);
  config.fields = {
    {"model", yaml["model"].as<std::string>()}, {"max_output_tokens", max_output}};
  if (const auto effort = yaml["reasoning_effort"]) {
    config.fields["reasoning"] = {{"effort", effort.as<std::string>()}};
  }
  if (const auto parallel = yaml["parallel_tool_calls"]) {
    config.fields["parallel_tool_calls"] = parallel.as<bool>();
  }
  if (const auto cache_key = yaml["prompt_cache_key"]) {
    config.fields["prompt_cache_key"] = cache_key.as<std::string>();
  }
  if (const auto include = yaml["include"]) {
    config.fields["include"] = include.as<std::vector<std::string>>();
  }
  if (config.local_history) {
    if (const auto store = yaml["store"]) {
      config.fields["store"] = store.as<bool>();
    }
  } else {
    config.fields["store"] = true;
    config.fields["context_management"] = Json::array({
        {{"type", "compaction"}, {"compact_threshold", config.threshold}}});
  }
  return config;
}
}  // namespace

/** @brief 队列与网络互相独立；会话锁不跨越 send、interrupt、close 或 join。 */
struct SessionManager::Impl
{
  explicit Impl(const std::string & path)
  : config(readConfig(path)),
    loader("vln_agent", "vln_agent::ResponsesTransport"),
    transport(loader.createSharedInstance(config.transport))
  {
    if (config.stream_output) {
      stream = {{"response_id", ""}, {"state", "idle"}, {"content", Json::array()}};
    }
  }

  ~Impl() {shutdown();}

  void start()
  {
    transport->connect(config.base_url, config.api_key,
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
    if (!config.forward_tool_async) {
      for (auto & tool : request["tools"]) {tool.erase("async");}
    }
    if (config.local_history) {
      auto input = local_context;
      utils::appendInputs(input, pending_input);
      request["input"] = std::move(input);
    } else {
      request["input"] = pending_input;
      if (!latest_response_id.empty()) {request["previous_response_id"] = latest_response_id;}
    }
    return request;
  }

  /** @brief 摘要和导航轮共用调度；摘要期间保留待发输入。 */
  void prepareRequest()
  {
    const auto total_tokens = usage.find("total_tokens");
    if (config.local_history && !compacted_for_send && total_tokens != usage.end() &&
      total_tokens->is_number_integer() && total_tokens->get<double>() >= config.threshold)
    {
      const auto range = session::compactRange(local_context, config.recent_items);
      if (!range.empty()) {
        Json summary = {
          {"model", config.fields.at("model")},
          {"max_output_tokens", config.summary_tokens},
          {"input", session::summaryInput(local_context, range)}};
        if (const auto effort = config.fields.find("reasoning"); effort != config.fields.end()) {
          summary["reasoning"] = *effort;
        }
        if (config.fields.contains("store")) {summary["store"] = false;}
        summary_range = range;
        current_request = std::move(summary);
        return;
      }
    }
    if (!config.forward_tool_async) {
      auto & items = pending_input.get_ref<Json::array_t &>();
      // 先续接上一轮工具，再提交等待期间收到的其他输入。
      std::stable_partition(items.begin(), items.end(), [](const Json & item) {
          return item.value("type", std::string()) == "function_call_output";
        });
    }
    current_request = buildRequest();
    current_delta = std::move(pending_input);
    pending_input = Json::array();
    compacted_for_send = false;
  }

  /** @brief 在会话锁内记录会话错误，保留当前与待发输入。 */
  void fail(const std::string & code, const std::string & message, const Json & event = nullptr)
  {
    last_error = {{"code", code}, {"message", message}, {"event", event}};
    context_invalid = !config.local_history && code == "previous_response_not_found";
    in_flight = false;
    last_finished = Clock::now();
    send_requested = false;
    if (config.stream_output && summary_range.empty()) {stream["state"] = "error";}
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
        if (config.stream_output && summary_range.empty()) {stream["state"] = "interrupted";}
      }
    }
    condition.notify_all();
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

  /** @brief 验证终态后提交会话轮次，摘要不会进入导航结果槽。 */
  void finish(const Json & response)
  {
    const auto id = response.at("id").get<std::string>();
    if (!active_response_id.empty() && id != active_response_id) {return;}
    const auto & output = response.at("output");
    if (id.empty() || response.at("status") != "completed" || !output.is_array()) {
      throw std::invalid_argument("完成事件缺少完整 response");
    }

    if (!summary_range.empty()) {
      session::installSummary(local_context, summary_range, output);
      summary_range = {};
      compacted_for_send = true;
      send_requested = true;
      active_response_id = latest_response_id;
      ++compaction_count;
    } else {
      std::unordered_set<std::string> tool_calls;
      if (!config.forward_tool_async) {
        for (const auto & item : output) {
          if (item.value("type", std::string()) == "function_call") {
            tool_calls.insert(item.at("call_id").get<std::string>());
          }
        }
      }
      if (config.stream_output) {finishText(response);}
      if (config.local_history) {
        // current_request 已通过输入校验；完成响应校验后才整体提交。
        auto & input = current_request.at("input");
        utils::appendInputs(input, output);
        local_context = std::move(input);
      } else {
        compaction_count += std::count_if(output.begin(), output.end(), [](const Json & item) {
            return item.value("type", std::string()) == "compaction";
          });
      }
      latest_response_id = id;
      active_response_id = id;
      usage = response.value("usage", Json());
      awaiting_tool_outputs = std::move(tool_calls);
      response_slot = response;
    }
    current_delta = Json::array();
    current_request = nullptr;
    last_finished = Clock::now();
    in_flight = false;
  }

  void onEvent(const Json & event)
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (closed || !in_flight) {return;}
      const bool display = config.stream_output && summary_range.empty();
      try {
        const auto type = event.at("type").get<std::string>();
        if (type == "response.created" || type == "response.in_progress") {
          active_response_id = event.at("response").at("id").get<std::string>();
          if (display) {stream["response_id"] = active_response_id;}
        } else if (type == "response.output_text.delta" || type == "response.refusal.delta") {
          if (display) {appendText(event);}
        } else if (type == "response.completed") {
          finish(event.at("response"));
        } else if (type == "error" || type == "response.failed" || type == "response.incomplete") {
          const auto & source = type == "error" ? event : event.at("response");
          const auto nested = source.find("error");
          const auto & error = nested != source.end() && nested->is_object() ? *nested : source;
          const auto code = error.find("code");
          const auto message = error.find("message");
          const std::string error_code =
            code != error.end() && code->is_string() ? code->get<std::string>() : type;
          const std::string error_message = message != error.end() && message->is_string() ?
            message->get<std::string>() : "模型请求未完整完成";
          fail(summary_range.empty() ? error_code : "CONTEXT_SUMMARY_FAILED", error_message, event);
        }
      } catch (const std::exception & error) {
        fail(summary_range.empty() ? "INVALID_RESPONSE" : "CONTEXT_SUMMARY_FAILED", error.what(), event);
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
      const auto timeout = summary_range.empty() ? config.timeout : std::chrono::seconds(30);
      if (in_flight && timeout != Clock::duration::zero() && now >= last_started + timeout)
      {
        fail(summary_range.empty() ? "REQUEST_TIMEOUT" : "CONTEXT_SUMMARY_TIMEOUT",
          "等待模型响应超时");
        // 在换连接完成前关闭发送入口，隔离旧轮的迟到事件。
        available = false;
        lock.unlock();
        transport->interrupt();
        lock.lock();
        continue;
      }
      const bool eligible = send_requested && !pending_input.empty() &&
        !in_flight && response_slot.is_null() && last_error.is_null() && available &&
        awaiting_tool_outputs.empty();
      const auto allowed_at = ever_started ? last_started + config.interval : now;
      if (eligible && now >= allowed_at) {
        send_requested = false;
        try {
          prepareRequest();
        } catch (const std::exception & error) {
          fail("INVALID_REQUEST", error.what());
        }
        if (!last_error.is_null()) {continue;}
        in_flight = true;
        last_started = now;
        last_finished = {};
        ever_started = true;
        active_response_id.clear();
        if (config.stream_output && summary_range.empty()) {
          stream = {{"response_id", ""}, {"state", "generating"}, {"content", Json::array()}};
        }
        auto frame = current_request;
        lock.unlock();
        try {
          transport->send(std::move(frame));
        } catch (const std::exception & error) {
          lock.lock();
          fail("TRANSPORT_SEND_FAILED", error.what());
          continue;
        }
        lock.lock();
        continue;
      }
      auto wake = Clock::time_point::max();
      if (in_flight && timeout != Clock::duration::zero()) {
        wake = last_started + timeout;
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
    if (in_flight) {return summary_range.empty() ? "generating" : "compacting";}
    if (!awaiting_tool_outputs.empty()) {return "waiting_tool_outputs";}
    if (send_requested && ever_started && Clock::now() < last_started + config.interval) {
      return "waiting_interval";
    }
    return "idle";
  }

  /** @brief 新输入追加成功后，按 call_id 解除对下一轮请求的等待。 */
  void acceptToolOutputs(std::size_t first)
  {
    if (awaiting_tool_outputs.empty()) {return;}
    const auto & items = pending_input.get_ref<const Json::array_t &>();
    for (auto it = items.begin() + first; it != items.end(); ++it) {
      if (it->value("type", std::string()) == "function_call_output") {
        awaiting_tool_outputs.erase(it->at("call_id").get_ref<const std::string &>());
      }
    }
    if (awaiting_tool_outputs.empty()) {condition.notify_all();}
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
    transport->close();
    if (scheduler.joinable()) {
      scheduler.join();
    }
    std::lock_guard<std::mutex> lock(mutex);
    current_request = nullptr;
    pending_input = Json::array();
    awaiting_tool_outputs.clear();
    current_delta = Json::array();
    summary_range = {};
    local_context = Json::array();
    response_slot = nullptr;
    in_flight = false;
    last_error = nullptr;
    usage = nullptr;
    latest_response_id.clear();
    active_response_id.clear();
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
  std::unordered_set<std::string> awaiting_tool_outputs;  ///< 同步模式尚未收到结果的调用。
  Json response_slot;                      ///< 唯一完整结果槽。
  Json local_context = Json::array();       ///< 本地模式成功提交的有效上下文。
  Json current_delta = Json::array();       ///< 当前业务轮的新输入，重试时独立恢复。
  session::ContextRange summary_range;      ///< 非空表示当前请求为摘要。
  bool compacted_for_send{false};            ///< 同一轮业务发送前最多进行一次摘要。
  Json last_error, usage, stream;           ///< 会话错误、最近用量和可选显示。
  std::string latest_response_id, active_response_id;
  std::uint64_t compaction_count{0};
  bool closed{false}, available{false}, in_flight{false}, send_requested{false};
  bool ever_started{false}, context_invalid{false};
  Clock::time_point last_started{}, last_finished{};  ///< 本轮起止时刻，用于稳定的耗时查询。
  pluginlib::ClassLoader<ResponsesTransport> loader;
  std::shared_ptr<ResponsesTransport> transport;
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
  const auto first = impl_->pending_input.size();
  utils::appendInputs(impl_->pending_input, input);
  impl_->acceptToolOutputs(first);
}

void SessionManager::addInputs(const Json::array_t & input_items)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->ensureOpen();
  const auto first = impl_->pending_input.size();
  utils::appendInputs(impl_->pending_input, input_items);
  impl_->acceptToolOutputs(first);
}

void SessionManager::request()
{
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->ensureOpen();
    if (impl_->context_invalid) {
      return;
    }
    if (!impl_->in_flight && (!impl_->current_request.is_null() || !impl_->last_error.is_null())) {
      utils::prependArray(impl_->pending_input, impl_->current_delta);
      impl_->current_delta = Json::array();
      impl_->summary_range = {};
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
      !impl_->closed && !impl_->in_flight &&
      (!impl_->current_request.is_null() || !impl_->last_error.is_null());
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
  status["transport"] = impl_->transport->getStatus();
  return status;
}

Json SessionManager::getContextStatus() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return {
    {"latest_response_id", impl_->latest_response_id},
    {"mode", impl_->config.local_history ? "local" : "server"},
    {"history_item_count", impl_->local_context.size()},
    {"budget_tokens", impl_->config.budget}, {"compact_threshold", impl_->config.threshold},
    {"compaction_in_flight", impl_->in_flight && !impl_->summary_range.empty()},
    {"usage", impl_->usage}, {"compaction_count", impl_->compaction_count}};
}

Json SessionManager::getStreamSnapshot() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->stream;
}

void SessionManager::shutdown() {impl_->shutdown();}

}  // namespace vln_agent
