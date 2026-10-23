#include "ws_fixture.hpp"

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/websocket/ssl.hpp>
#include <openssl/pem.h>
#include <openssl/x509v3.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <limits>
#include <set>
#include <stdexcept>
#include <thread>

namespace vln_agent::test
{
namespace net = boost::asio;
namespace beast = boost::beast;
namespace ws = beast::websocket;
using tcp = net::ip::tcp;

FixtureFiles::FixtureFiles()
{
  static std::atomic_int counter{0};
  directory = (std::filesystem::path(VLN_TEST_OUTPUT_DIR) /
    ("session_" + std::to_string(getpid()) + "_" + std::to_string(++counter))).string();
  std::filesystem::create_directories(directory);
}

std::string FixtureFiles::write(const std::string & name, const std::string & text) const
{
  const auto path = (std::filesystem::path(directory) / name).string();
  std::ofstream(path) << text;
  return path;
}

std::string FixtureFiles::sessionConfig(const std::string & url, const Json & overrides) const
{
  Json config = {
    {"base_url", url}, {"api_key", "loopback-fixture"}, {"model", "gpt-6-luna"},
    {"max_request_hz", 100.0}, {"request_timeout_sec", 0}, {"stream_output", false},
    {"context", {{"budget_tokens", 10000}, {"compact_ratio", 0.8}}}};
  config.update(overrides);
  return write("agent.yaml", Json{{"tool_manager", {{"ignored", true}}},
      {"session_manager", config}}.dump(2));
}

Json message(const std::string & text, const std::string & role)
{
  return {{"role", role}, {"content", Json::array({{{"type", "input_text"}, {"text", text}}})}};
}

Json completed(const std::string & id, const std::string & text)
{
  return {{"id", id}, {"status", "completed"},
    {"output", Json::array({
          {{"type", "reasoning"}, {"encrypted_content", "原样保留的密文"}},
          {{"type", "message"}, {"role", "assistant"},
            {"content", Json::array({{{"type", "output_text"}, {"text", text}}})}}})},
    {"usage", {{"input_tokens", 100}, {"output_tokens", 20},
        {"input_tokens_details", {{"cached_tokens", 64}}}}},
    {"metadata", {{"original", true}}}};
}

void waitFor(const std::function<bool()> & predicate)
{
  const auto end = std::chrono::steady_clock::now() + 3s;
  while (!predicate()) {
    if (std::chrono::steady_clock::now() >= end) {
      throw std::runtime_error("回环协议测试等待超过三秒");
    }
    std::this_thread::sleep_for(2ms);
  }
}

/** @brief 随机回环端口、单连接同步服务线程和测试门闩。 */
struct WsFixture::Impl
{
  Impl(Handler handler_value, WsServerOptions options_value)
  : handler(std::move(handler_value)), options(std::move(options_value)),
    tls_context(net::ssl::context::tls_server)
  {
    if (options.tls) {
      tls_context.use_certificate_chain_file(options.certificate);
      tls_context.use_private_key_file(options.private_key, net::ssl::context::pem);
    }
    listener = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (listener < 0 || ::bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof(address)) ||
      ::listen(listener, 8))
    {
      throw std::runtime_error("无法创建回环测试监听端口");
    }
    socklen_t length = sizeof(address);
    getsockname(listener, reinterpret_cast<sockaddr *>(&address), &length);
    port = ntohs(address.sin_port);
    thread = std::thread([this] {serve();});
  }

  template<class Stream>
  void process(Stream & stream, int native_socket)
  {
    try {
      beast::flat_buffer buffer;
      beast::http::request<beast::http::string_body> upgrade;
      beast::http::read(stream.next_layer(), buffer, upgrade);
      {
        std::lock_guard<std::mutex> lock(mutex);
        path_values.push_back(std::string(upgrade.target()));
        authorization_values.push_back(std::string(upgrade[beast::http::field::authorization]));
      }
      if (options.reject_upgrade) {
        beast::http::response<beast::http::string_body> rejected{
          beast::http::status::unauthorized, 11};
        rejected.body() = "fixture rejection";
        rejected.prepare_payload();
        beast::http::write(stream.next_layer(), rejected);
        return;
      }
      stream.accept(upgrade);
      {
        std::lock_guard<std::mutex> lock(mutex);
        ++connection_count;
      }
      stream.text(true);
      Peer peer{
        [&](const std::string & text, bool fragment) {
          if (fragment) {
            stream.auto_fragment(false);
            const auto split = text.size() / 2;
            stream.write_some(false, net::buffer(text.data(), split));
            stream.write_some(true, net::buffer(text.data() + split, text.size() - split));
          } else {
            stream.write(net::buffer(text));
          }
        },
        [native_socket] {::shutdown(native_socket, SHUT_RDWR);}};
      while (!stopping.load()) {
        beast::flat_buffer input;
        stream.read(input);
        const auto request = Json::parse(beast::buffers_to_string(input.data()));
        std::size_t index;
        {
          std::lock_guard<std::mutex> lock(mutex);
          index = request_values.size();
          request_values.push_back(request);
          start_values.push_back(std::chrono::steady_clock::now());
        }
        handler(peer, request, index);
      }
    } catch (const std::exception &) {
      // 客户端断开、TLS 拒绝、主动取消及测试结束均允许重新接受连接。
    }
  }

  void serve()
  {
    while (!stopping.load()) {
      const int fd = ::accept(listener, nullptr, nullptr);
      if (fd < 0) {
        return;
      }
      {
        std::lock_guard<std::mutex> lock(socket_mutex);
        active_socket = fd;
      }
      net::io_context io;
      tcp::socket socket(io);
      socket.assign(tcp::v4(), fd);
      if (options.stall_handshake) {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [this] {
          return stopping.load() || released.count(std::numeric_limits<std::size_t>::max()) != 0;
        });
      }
      if (stopping.load()) {
        std::lock_guard<std::mutex> lock(socket_mutex);
        active_socket = -1;
        continue;
      }
      if (options.tls) {
        ws::stream<net::ssl::stream<tcp::socket>> stream(std::move(socket), tls_context);
        try {
          stream.next_layer().handshake(net::ssl::stream_base::server);
          process(stream, fd);
        } catch (const std::exception &) {
        }
        std::lock_guard<std::mutex> lock(socket_mutex);
        active_socket = -1;
      } else {
        ws::stream<tcp::socket> stream(std::move(socket));
        process(stream, fd);
        std::lock_guard<std::mutex> lock(socket_mutex);
        active_socket = -1;
      }
      std::lock_guard<std::mutex> lock(socket_mutex);
      active_socket = -1;
    }
  }

  Handler handler;
  WsServerOptions options;
  net::ssl::context tls_context;
  int listener{-1}, port{0}, active_socket{-1};
  std::atomic_bool stopping{false};
  mutable std::mutex mutex;
  std::mutex socket_mutex;
  std::condition_variable condition;
  std::set<std::size_t> released;
  std::vector<Json> request_values;
  std::vector<std::chrono::steady_clock::time_point> start_values;
  std::vector<std::string> path_values, authorization_values;
  std::size_t connection_count{0};
  std::thread thread;
};

WsFixture::WsFixture(Handler handler, WsServerOptions options)
: impl_(std::make_unique<Impl>(std::move(handler), std::move(options)))
{
}

WsFixture::~WsFixture()
{
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stopping.store(true);
  }
  impl_->condition.notify_all();
  ::shutdown(impl_->listener, SHUT_RDWR);
  ::close(impl_->listener);
  {
    std::lock_guard<std::mutex> lock(impl_->socket_mutex);
    if (impl_->active_socket >= 0) {
      ::shutdown(impl_->active_socket, SHUT_RDWR);
    }
  }
  impl_->thread.join();
}

std::string WsFixture::url(const std::string & host) const
{
  return std::string(impl_->options.tls ? "https://" : "http://") +
         host + ":" + std::to_string(impl_->port) + "/v1";
}

std::vector<Json> WsFixture::requests() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->request_values;
}

std::vector<std::chrono::steady_clock::time_point> WsFixture::starts() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->start_values;
}

std::vector<std::string> WsFixture::paths() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->path_values;
}

std::vector<std::string> WsFixture::authorizations() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->authorization_values;
}

std::size_t WsFixture::connections() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->connection_count;
}

void WsFixture::release(std::size_t index)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->released.insert(index);
  impl_->condition.notify_all();
}

void WsFixture::waitRelease(std::size_t index)
{
  std::unique_lock<std::mutex> lock(impl_->mutex);
  impl_->condition.wait(lock, [&] {
    return impl_->stopping.load() || impl_->released.count(index) != 0;
  });
}

WsServerOptions testCertificate(const FixtureFiles & files)
{
  std::unique_ptr<EVP_PKEY_CTX, decltype(&EVP_PKEY_CTX_free)> generator(
    EVP_PKEY_CTX_new_id(EVP_PKEY_RSA, nullptr), EVP_PKEY_CTX_free);
  EVP_PKEY_keygen_init(generator.get());
  EVP_PKEY_CTX_set_rsa_keygen_bits(generator.get(), 2048);
  EVP_PKEY * raw_key = nullptr;
  if (EVP_PKEY_keygen(generator.get(), &raw_key) != 1) {
    throw std::runtime_error("无法生成测试 TLS 密钥");
  }
  std::unique_ptr<EVP_PKEY, decltype(&EVP_PKEY_free)> key(raw_key, EVP_PKEY_free);
  std::unique_ptr<X509, decltype(&X509_free)> certificate(X509_new(), X509_free);
  X509_set_version(certificate.get(), 2);
  ASN1_INTEGER_set(X509_get_serialNumber(certificate.get()), 1);
  X509_gmtime_adj(X509_getm_notBefore(certificate.get()), -60);
  X509_gmtime_adj(X509_getm_notAfter(certificate.get()), 3600);
  X509_set_pubkey(certificate.get(), key.get());
  auto * name = X509_get_subject_name(certificate.get());
  X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC,
    reinterpret_cast<const unsigned char *>("localhost"), -1, -1, 0);
  X509_set_issuer_name(certificate.get(), name);
  for (const auto & extension : {
      std::pair<int, const char *>{NID_subject_alt_name, "DNS:localhost"},
      {NID_basic_constraints, "critical,CA:TRUE"}})
  {
    auto * value = X509V3_EXT_conf_nid(nullptr, nullptr, extension.first,
      const_cast<char *>(extension.second));
    X509_add_ext(certificate.get(), value, -1);
    X509_EXTENSION_free(value);
  }
  X509_sign(certificate.get(), key.get(), EVP_sha256());
  WsServerOptions options;
  options.tls = true;
  options.certificate = files.directory + "/localhost.pem";
  options.private_key = files.directory + "/localhost.key";
  auto * cert_file = fopen(options.certificate.c_str(), "w");
  PEM_write_X509(cert_file, certificate.get());
  fclose(cert_file);
  auto * key_file = fopen(options.private_key.c_str(), "w");
  PEM_write_PrivateKey(key_file, key.get(), nullptr, nullptr, 0, nullptr, nullptr);
  fclose(key_file);
  return options;
}

}  // namespace vln_agent::test
