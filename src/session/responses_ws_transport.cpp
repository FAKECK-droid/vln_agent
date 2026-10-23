#include "vln_agent/session/responses_ws_transport.hpp"

#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/websocket/ssl.hpp>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "vln_agent/utils/websocket_url.hpp"

namespace vln_agent
{
namespace net = boost::asio;
namespace beast = boost::beast;
namespace ws = beast::websocket;
namespace ssl = net::ssl;
using tcp = net::ip::tcp;
using Clock = std::chrono::steady_clock;

/** @brief 所有 socket 操作由同一网络线程推进；公开方法只提交操作或读取快照。 */
struct ResponsesWsTransport::Impl
{
  using PlainSocket = ws::stream<beast::tcp_stream>;
  using TlsSocket = ws::stream<beast::ssl_stream<beast::tcp_stream>>;

  explicit Impl(WsTransportOptions value)
  : options(value), resolver(io), tls_context(ssl::context::tls_client), write_timer(io)
  {
    if (options.connect_timeout.count() <= 0 || options.write_timeout.count() <= 0 ||
      options.idle_timeout.count() <= 0 || options.reconnect_delay.count() <= 0 ||
      options.max_reconnect_delay < options.reconnect_delay)
    {
      throw std::invalid_argument("网络超时及重连间隔必须为正数，退避上限不能小于初始值");
    }
  }

  /** @brief 更新状态后释放状态锁，再通知上层，避免锁跨越回调边界。 */
  void state(const std::string & name, bool available, const Json & error = nullptr)
  {
    Json snapshot;
    {
      std::lock_guard<std::mutex> lock(status_mutex);
      status["state"] = name;
      status["connected"] = available;
      status["connection_id"] = generation;
      status["reconnect_attempts"] = retries;
      if (!error.is_null()) {
        status["last_error"] = error;
      }
      snapshot = status;
    }
    if (on_state) {
      on_state(snapshot);
    }
  }

  Json getStatus() const
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    return status;
  }

  /** @brief 取消保活定时器并中断底层 I/O，异步回调仍由网络线程排空。 */
  void closeSockets()
  {
    resolver.cancel();
    boost::system::error_code ignored;
    write_timer.cancel(ignored);
    const auto close = [&](auto & socket) {
        socket.set_option(ws::stream_base::timeout{
            ws::stream_base::none(), ws::stream_base::none(), false});
        beast::get_lowest_layer(socket).socket().close(ignored);
      };
    if (plain) {
      close(*plain);
    }
    if (tls) {
      close(*tls);
    }
  }

  /** @brief 标记网络故障；不伪造 Responses error 事件。 */
  void fail(const std::string & stage, const std::string & message)
  {
    if (broken || stopping.load()) {
      return;
    }
    broken = true;
    state("reconnecting", false, Json{{"stage", stage}, {"message", message}});
    closeSockets();
  }

  /** @brief 复用旧 WS 实现的异步阶段等待，统一覆盖 DNS、TCP、TLS 和 Upgrade。 */
  template<class Start>
  void wait(Start start, Clock::time_point deadline)
  {
    bool done = false;
    boost::system::error_code error;
    io.restart();
    start([&](boost::system::error_code result, auto...) {
        error = result;
        done = true;
      });
    while (!done) {
      io.restart();
      io.run_for(std::chrono::milliseconds(10));
      if (!done && (stopping.load() || broken || Clock::now() >= deadline)) {
        closeSockets();
        while (!done) {
          io.restart();
          io.run_for(std::chrono::milliseconds(10));
        }
        throw std::runtime_error("网络操作已中断或超时");
      }
    }
    if (error) {
      throw boost::system::system_error(error);
    }
  }

  template<class Function>
  void withSocket(Function function)
  {
    if (url.secure) {
      function(*tls);
    } else {
      function(*plain);
    }
  }

  template<class Socket>
  void configure(Socket & socket)
  {
    socket.set_option(ws::stream_base::timeout{
        options.connect_timeout, options.idle_timeout, true});
    socket.set_option(ws::stream_base::decorator([this](ws::request_type & request) {
        request.set(beast::http::field::authorization, "Bearer " + api_key);
        request.set(beast::http::field::user_agent, "vln_agent/0.1");
      }));
    socket.text(true);
    socket.read_message_max(64 * 1024 * 1024);
  }

  /** @brief 建立一条经过系统 CA 和主机名验证的连接。 */
  void establish()
  {
    const auto deadline = Clock::now() + options.connect_timeout;
    tcp::resolver::results_type endpoints;
    phase = "dns";
    wait([&](auto done) {
        resolver.async_resolve(url.host, url.port,
          [&, done](boost::system::error_code error, tcp::resolver::results_type result) {
            endpoints = std::move(result);
            done(error);
          });
      }, deadline);
    if (url.secure) {
      tls = std::make_unique<TlsSocket>(io, tls_context);
      tls->next_layer().set_verify_callback(ssl::host_name_verification(url.host));
      if (!SSL_set_tlsext_host_name(tls->next_layer().native_handle(), url.host.c_str())) {
        throw std::runtime_error("无法设置 TLS 主机名");
      }
    } else {
      plain = std::make_unique<PlainSocket>(io);
    }
    phase = "tcp";
    withSocket([&](auto & socket) {
        configure(socket);
        wait([&](auto done) {
            beast::get_lowest_layer(socket).async_connect(endpoints, done);
          }, deadline);
      });
    if (url.secure) {
      phase = "tls";
      wait([&](auto done) {
          tls->next_layer().async_handshake(ssl::stream_base::client, done);
        }, deadline);
    }
    phase = "websocket";
    withSocket([&](auto & socket) {
        wait([&](auto done) {
            socket.async_handshake(url.host_header, url.target, done);
          }, deadline);
      });
  }

  /** @brief 始终保持读取，处理分帧、探活及 JSON 解码，业务事件原样上交。 */
  void read()
  {
    withSocket([&](auto & socket) {
        socket.async_read(incoming, [this](boost::system::error_code error, std::size_t) {
            if (broken || stopping.load()) {
              return;
            }
            if (error) {
              fail("read", error.message());
              return;
            }
            Json event;
            try {
              bool text = false;
              withSocket([&](auto & socket) {text = socket.got_text();});
              if (!text) {
                throw std::runtime_error("Responses 通道只接受 JSON 文本帧");
              }
              event = Json::parse(beast::buffers_to_string(incoming.data()));
              incoming.consume(incoming.size());
              if (!event.is_object()) {
                throw std::runtime_error("Responses 事件必须为 JSON 对象");
              }
            } catch (const std::exception & error) {
              fail("decode", error.what());
              return;
            }
            on_event(event);
            if (!broken && !stopping.load()) {
              read();
            }
          });
      });
  }

  /** @brief 只保存当前异步写缓冲；连接代号变化后原帧失效。 */
  void write(const Json & event, std::uint64_t expected_generation, bool was_connected)
  {
    if (stopping.load()) {
      return;
    }
    if (!was_connected || broken || expected_generation != generation) {
      if (!broken && getStatus().at("connected").get<bool>()) {
        fail("send", "提交期间连接已改变，请由上层显式重试");
      } else if (on_state) {
        on_state(getStatus());
      }
      return;
    }
    if (writing) {
      fail("send", "上一帧仍在写入");
      return;
    }
    std::shared_ptr<std::string> bytes;
    try {
      bytes = std::make_shared<std::string>(event.dump());
    } catch (const std::exception & error) {
      fail("encode", error.what());
      return;
    }
    writing = true;
    write_timer.expires_after(options.write_timeout);
    write_timer.async_wait([this](boost::system::error_code error) {
        if (!error && writing) {
          fail("write", "WebSocket 写入超时");
        }
      });
    withSocket([&](auto & socket) {
        socket.async_write(net::buffer(*bytes),
          [this, bytes](boost::system::error_code error, std::size_t) {
            writing = false;
            boost::system::error_code ignored;
            write_timer.cancel(ignored);
            if (error) {
              fail("write", error.message());
            }
          });
      });
  }

  /** @brief 在同一线程排空旧连接回调后重连，不重放任何业务内容。 */
  void run()
  {
    auto delay = options.reconnect_delay;
    while (!stopping.load()) {
      broken = false;
      try {
        state(retries == 0 ? "connecting" : "reconnecting", false);
        establish();
        if (!stopping.load() && !broken) {
          ++generation;
          state("connected", true);
          delay = options.reconnect_delay;
          read();
          while (!stopping.load() && !broken) {
            io.restart();
            io.run_for(std::chrono::milliseconds(20));
          }
        }
      } catch (const std::exception & error) {
        if (!stopping.load() && !broken) {
          fail(phase, error.what());
        }
      }
      broken = true;
      closeSockets();
      io.restart();
      io.run();
      plain.reset();
      tls.reset();
      incoming.consume(incoming.size());
      writing = false;
      if (stopping.load()) {
        break;
      }
      ++retries;
      state("reconnecting", false);
      std::unique_lock<std::mutex> lock(retry_mutex);
      retry_condition.wait_for(lock, delay, [this] {return stopping.load();});
      delay = std::min(delay * 2, options.max_reconnect_delay);
    }
    state("closed", false);
  }

  WsTransportOptions options;               ///< 固定网络策略，与 YAML 和模型配置解耦。
  net::io_context io;
  tcp::resolver resolver;
  ssl::context tls_context;
  net::steady_timer write_timer;
  std::unique_ptr<PlainSocket> plain;
  std::unique_ptr<TlsSocket> tls;
  beast::flat_buffer incoming;
  utils::WebSocketUrl url;
  std::string api_key, phase;
  EventHandler on_event;
  StateHandler on_state;
  std::atomic_bool stopping{false};          ///< 主线程与网络线程共用的关闭标志。
  bool broken{false}, writing{false};         ///< 仅网络线程访问。
  std::uint64_t generation{0}, retries{0};   ///< 仅网络线程写入，公开值通过状态锁发布。
  mutable std::mutex status_mutex;
  Json status{{"state", "closed"}, {"connected", false}, {"connection_id", 0},
    {"reconnect_attempts", 0}, {"last_error", nullptr}};
  std::mutex lifecycle_mutex;               ///< 串行化 connect/send/interrupt/close 的入口。
  std::mutex retry_mutex;
  std::condition_variable retry_condition;
  std::thread thread;
  bool started{false};                      ///< 在生命周期锁内访问，不支持重新打开已关闭实例。
};

ResponsesWsTransport::ResponsesWsTransport(WsTransportOptions options)
: impl_(std::make_unique<Impl>(options))
{
}

ResponsesWsTransport::~ResponsesWsTransport() {close();}

void ResponsesWsTransport::connect(
  const std::string & base_url, const std::string & api_key,
  EventHandler on_event, StateHandler on_state)
{
  std::lock_guard<std::mutex> lock(impl_->lifecycle_mutex);
  if (impl_->started || impl_->stopping.load()) {
    throw std::logic_error("ResponsesWsTransport 每个实例只连接一次");
  }
  impl_->url = utils::parseWebSocketUrl(base_url);
  impl_->api_key = api_key;
  impl_->on_event = std::move(on_event);
  impl_->on_state = std::move(on_state);
  if (impl_->url.secure) {
    impl_->tls_context.set_default_verify_paths();
    impl_->tls_context.set_verify_mode(ssl::verify_peer);
  }
  impl_->started = true;
  impl_->thread = std::thread([this] {impl_->run();});
}

void ResponsesWsTransport::send(Json event)
{
  std::lock_guard<std::mutex> lock(impl_->lifecycle_mutex);
  if (impl_->stopping.load()) {
    return;
  }
  if (!impl_->started) {
    throw std::logic_error("ResponsesWsTransport 尚未连接");
  }
  const auto status = impl_->getStatus();
  const auto generation = status.at("connection_id").get<std::uint64_t>();
  const bool connected = status.at("connected").get<bool>();
  net::post(impl_->io, [impl = impl_.get(), event = std::move(event), generation, connected] {
      impl->write(event, generation, connected);
    });
}

bool ResponsesWsTransport::connected() const
{
  return getStatus().at("connected").get<bool>();
}

Json ResponsesWsTransport::getStatus() const {return impl_->getStatus();}

void ResponsesWsTransport::interrupt()
{
  std::lock_guard<std::mutex> lock(impl_->lifecycle_mutex);
  if (impl_->stopping.load()) {
    return;
  }
  net::post(impl_->io, [impl = impl_.get()] {
      impl->broken = true;
      impl->state("reconnecting", false);
      impl->closeSockets();
    });
}

void ResponsesWsTransport::close()
{
  std::lock_guard<std::mutex> lock(impl_->lifecycle_mutex);
  {
    std::lock_guard<std::mutex> retry_lock(impl_->retry_mutex);
    if (impl_->stopping.exchange(true)) {
      return;
    }
  }
  impl_->retry_condition.notify_all();
  if (impl_->thread.joinable()) {
    impl_->thread.join();
  }
}

}  // namespace vln_agent
