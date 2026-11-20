#include "vln_agent/session/responses_http_transport.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

#include <curl/curl.h>
#include <pluginlib/class_list_macros.hpp>

#include "vln_agent/utils/websocket_url.hpp"

namespace vln_agent
{
namespace
{
constexpr std::size_t max_event_bytes = 16 * 1024 * 1024;

void checkMulti(CURLMcode result)
{
  if (result != CURLM_OK) {throw std::runtime_error(curl_multi_strerror(result));}
}

/** @brief 保留未完成的行，按 SSE 空行分隔事件；支持 CRLF 和多行 data。 */
struct EventStream
{
  const ResponsesTransport::EventHandler & deliver;
  std::string line, data, event_name;
  bool terminal{false};

  void event(Json value)
  {
    if (!value.is_object()) {throw std::invalid_argument("Responses 事件必须是 JSON 对象");}
    if (!value.contains("type") && !event_name.empty()) {value["type"] = event_name;}
    const auto & type = value.at("type").get_ref<const std::string &>();
    terminal = type == "response.completed" || type == "response.failed" ||
      type == "response.incomplete" || type == "error";
    deliver(value);
  }

  void feed(std::string_view bytes)
  {
    while (!bytes.empty() && !terminal) {
      const auto end = bytes.find('\n');
      line.append(bytes.substr(0, end));
      if (line.size() + data.size() + event_name.size() > max_event_bytes) {
        throw std::length_error("HTTP SSE 事件超过大小上限");
      }
      if (end == std::string_view::npos) {return;}
      bytes.remove_prefix(end + 1);
      if (!line.empty() && line.back() == '\r') {line.pop_back();}
      if (line.empty()) {
        if (!data.empty() && data != "[DONE]") {event(Json::parse(data));}
        data.clear();
        event_name.clear();
      } else if (line.rfind("data:", 0) == 0) {
        if (!data.empty()) {data.push_back('\n');}
        data.append(line, line.size() > 5 && line[5] == ' ' ? 6 : 5);
      } else if (line.rfind("event:", 0) == 0) {
        event_name.assign(line, line.size() > 6 && line[6] == ' ' ? 7 : 6);
      }
      line.clear();
    }
  }
};

struct Download
{
  CURL * easy;
  EventStream events;
  std::string body, error;

  static std::size_t write(char * bytes, std::size_t size, std::size_t count, void * user)
  {
    auto & self = *static_cast<Download *>(user);
    const auto length = size * count;
    try {
      char * content_type = nullptr;
      curl_easy_getinfo(self.easy, CURLINFO_CONTENT_TYPE, &content_type);
      if (content_type && std::string_view(content_type).find("text/event-stream") == 0) {
        self.events.feed({bytes, length});
      } else {
        if (self.body.size() + length > max_event_bytes) {
          throw std::length_error("HTTP 响应超过大小上限");
        }
        self.body.append(bytes, length);
      }
      return length;
    } catch (const std::exception & e) {
      self.error = e.what();
      return 0;
    }
  }

  void finish(long status)
  {
    if (events.terminal) {return;}
    if (status >= 300) {
      auto response = Json::parse(body, nullptr, false);
      auto error = response.is_object() ? response.value("error", Json()) : Json();
      if (!error.is_object()) {error = Json::object();}
      if (!error.contains("code") || !error.at("code").is_string()) {
        error["code"] = "HTTP_" + std::to_string(status);
      }
      if (!error.contains("message") || !error.at("message").is_string()) {
        error["message"] = "HTTP 请求失败: " + std::to_string(status);
      }
      events.event({{"type", "error"}, {"error", std::move(error)}});
      return;
    }
    // 某些 Responses 服务直接返回 JSON，即使请求打开了 stream。
    if (!body.empty()) {
      auto response = Json::parse(body);
      const auto state = response.at("status").get<std::string>();
      if (state == "completed" || state == "failed" || state == "incomplete") {
        events.event({{"type", "response." + state}, {"response", std::move(response)}});
      }
    }
    if (!events.terminal) {throw std::runtime_error("HTTP 响应缺少终态事件");}
  }
};
}  // namespace

struct ResponsesHttpTransport::Impl
{
  void state(const char * name, bool ready, Json error = nullptr)
  {
    Json snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex);
      status["state"] = name;
      status["connected"] = ready;
      status["connection_id"] = generation.load() + 1;
      if (!error.is_null()) {status["last_error"] = std::move(error);}
      snapshot = status;
    }
    on_state(snapshot);
  }

  bool active(std::uint64_t id) const {return !stopping && generation == id;}

  void post(CURLM * multi, Json request, std::uint64_t id)
  {
    std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> easy(curl_easy_init(), curl_easy_cleanup);
    if (!easy) {throw std::runtime_error("无法初始化 HTTP 请求");}

    request["stream"] = true;
    const auto body = request.dump();
    const auto auth = "Authorization: Bearer " + api_key;
    std::unique_ptr<curl_slist, decltype(&curl_slist_free_all)> headers(nullptr, curl_slist_free_all);
    for (const auto * value : {auth.c_str(), "Content-Type: application/json", "Accept: text/event-stream"}) {
      auto * appended = curl_slist_append(headers.get(), value);
      if (!appended) {throw std::bad_alloc();}
      headers.release();
      headers.reset(appended);
    }

    const EventHandler receive = [&](const Json & event) {
        if (active(id)) {on_event(event);}
      };
    Download download{easy.get(), {receive, {}, {}, {}, false}, {}, {}};
    curl_easy_setopt(easy.get(), CURLOPT_URL, url.c_str());
    curl_easy_setopt(easy.get(), CURLOPT_HTTPHEADER, headers.get());
    curl_easy_setopt(easy.get(), CURLOPT_POSTFIELDS, body.data());
    curl_easy_setopt(easy.get(), CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(body.size()));
    curl_easy_setopt(easy.get(), CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(easy.get(), CURLOPT_CONNECTTIMEOUT_MS, 10000L);
    curl_easy_setopt(easy.get(), CURLOPT_WRITEFUNCTION, &Download::write);
    curl_easy_setopt(easy.get(), CURLOPT_WRITEDATA, &download);

    checkMulti(curl_multi_add_handle(multi, easy.get()));
    struct Remove
    {
      CURLM * multi;
      CURL * easy;
      ~Remove() {curl_multi_remove_handle(multi, easy);}
    } remove{multi, easy.get()};
    int running = 0;
    do {
      checkMulti(curl_multi_perform(multi, &running));
      if (!running || download.events.terminal || !active(id)) {break;}
      checkMulti(curl_multi_poll(multi, nullptr, 0, 100, nullptr));
    } while (active(id));
    int remaining = 0;
    while (auto * message = curl_multi_info_read(multi, &remaining)) {
      if (active(id) && !download.events.terminal && message->msg == CURLMSG_DONE &&
        message->data.result != CURLE_OK)
      {
        throw std::runtime_error(download.error.empty() ?
          curl_easy_strerror(message->data.result) : download.error);
      }
    }
    if (active(id)) {
      long code = 0;
      curl_easy_getinfo(easy.get(), CURLINFO_RESPONSE_CODE, &code);
      download.finish(code);
    }
  }

  void run()
  {
    std::unique_ptr<CURLM, decltype(&curl_multi_cleanup)> multi(curl_multi_init(), curl_multi_cleanup);
    if (!multi) {
      state("error", false, {{"stage", "connect"}, {"message", "无法初始化 HTTP 通道"}});
      return;
    }
    state("ready", true);
    while (!stopping) {
      Json request;
      std::uint64_t id = 0;
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [&] {return stopping || reset || pending.has_value();});
        if (stopping) {break;}
        if (reset) {
          reset = false;
          lock.unlock();
          state("ready", true);
          continue;
        }
        request = std::move(*pending);
        pending.reset();
        id = generation.load();
      }
      try {
        post(multi.get(), std::move(request), id);
      } catch (const std::exception & error) {
        if (active(id)) {
          state("interrupted", false, {{"stage", "http"}, {"message", error.what()}});
          state("ready", true);
        }
      }
    }
    state("closed", false);
  }

  mutable std::mutex mutex;
  std::mutex lifecycle;
  std::condition_variable condition;
  std::thread worker;
  std::string url, api_key;
  EventHandler on_event;
  StateHandler on_state;
  std::optional<Json> pending;
  std::atomic_bool stopping{false};
  std::atomic_uint64_t generation{0};
  bool reset{false}, started{false};
  Json status{{"state", "closed"}, {"connected", false}, {"connection_id", 0},
    {"reconnect_attempts", 0}, {"last_error", nullptr}};
};

ResponsesHttpTransport::ResponsesHttpTransport() : impl_(std::make_unique<Impl>()) {}
ResponsesHttpTransport::~ResponsesHttpTransport() {close();}

void ResponsesHttpTransport::connect(const std::string & base_url, const std::string & api_key,
  EventHandler on_event, StateHandler on_state)
{
  const bool https = base_url.rfind("https://", 0) == 0;
  if (!https && base_url.rfind("http://", 0) != 0) {
    throw std::invalid_argument("HTTP transport 必须使用 http:// 或 https://");
  }
  const auto endpoint = utils::parseWebSocketUrl(base_url);
  static const auto initialized = curl_global_init(CURL_GLOBAL_DEFAULT);
  if (initialized != CURLE_OK) {throw std::runtime_error(curl_easy_strerror(initialized));}
  std::lock_guard<std::mutex> lock(impl_->lifecycle);
  if (impl_->started || impl_->stopping) {
    throw std::logic_error("ResponsesHttpTransport 每个实例只启动一次");
  }
  impl_->url = std::string(https ? "https://" : "http://") + endpoint.host_header + endpoint.target;
  impl_->api_key = api_key;
  impl_->on_event = std::move(on_event);
  impl_->on_state = std::move(on_state);
  impl_->started = true;
  impl_->worker = std::thread([this] {impl_->run();});
}

void ResponsesHttpTransport::send(Json request)
{
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->stopping) {return;}
    if (impl_->pending) {throw std::logic_error("HTTP 已有待发送请求");}
    impl_->pending = std::move(request);
  }
  impl_->condition.notify_one();
}

Json ResponsesHttpTransport::getStatus() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->status;
}

void ResponsesHttpTransport::interrupt()
{
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    ++impl_->generation;
    impl_->pending.reset();
    impl_->reset = true;
  }
  impl_->condition.notify_one();
}

void ResponsesHttpTransport::close()
{
  std::lock_guard<std::mutex> lock(impl_->lifecycle);
  {
    std::lock_guard<std::mutex> state_lock(impl_->mutex);
    impl_->stopping = true;
    impl_->pending.reset();
  }
  impl_->condition.notify_all();
  if (impl_->worker.joinable()) {impl_->worker.join();}
}
}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::ResponsesHttpTransport, vln_agent::ResponsesTransport)
