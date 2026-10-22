#include <gtest/gtest.h>

#include <cstdlib>
#include <limits>
#include <mutex>
#include <optional>
#include <thread>

#include "ws_fixture.hpp"
#include "vln_agent/session/responses_ws_transport.hpp"

namespace vln_agent::test
{
namespace
{

/** @brief 捕获网络线程回调，测试读取时使用同一把锁。 */
class Observer
{
public:
  void event(const Json & value)
  {
    std::lock_guard<std::mutex> lock(mutex);
    events.push_back(value);
  }
  std::vector<Json> values()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return events;
  }
  void connect(ResponsesWsTransport & transport, const std::string & url)
  {
    transport.connect(url, "loopback-fixture",
      [this](const Json & value) {event(value);}, [](const Json &) {});
  }
private:
  std::mutex mutex;
  std::vector<Json> events;
};

/** @brief 仅当前测试进程临时增加测试 CA，离开作用域后恢复原环境。 */
class TestTrust
{
public:
  explicit TestTrust(const std::string & certificate)
  {
    if (const auto value = std::getenv("SSL_CERT_FILE")) {previous = value;}
    setenv("SSL_CERT_FILE", certificate.c_str(), 1);
  }
  ~TestTrust()
  {
    if (previous) {setenv("SSL_CERT_FILE", previous->c_str(), 1);}
    else {unsetenv("SSL_CERT_FILE");}
  }
private:
  std::optional<std::string> previous;
};

WsTransportOptions fastOptions()
{
  WsTransportOptions options;
  options.connect_timeout = 500ms;
  options.write_timeout = 500ms;
  options.idle_timeout = 1000ms;
  options.reconnect_delay = 20ms;
  options.max_reconnect_delay = 40ms;
  return options;
}

}  // namespace

TEST(WsTransport, AuthenticationFragmentedMessagesAndConnectionReuse)
{
  WsFixture server([](auto & peer, const Json &, std::size_t index) {
      peer.send(Json{{"type", "response.created"}, {"response", {{"id", "r1"}}}});
      peer.send(Json{{"type", "response.completed"},
          {"response", completed("r" + std::to_string(index + 1))}}, true);
    });
  Observer observer;
  ResponsesWsTransport transport(fastOptions());
  observer.connect(transport, server.url() + "/");
  waitFor([&] {return transport.connected();});
  transport.send(Json{{"type", "response.create"}, {"input", {message("first")}}});
  waitFor([&] {return observer.values().size() == 2;});
  transport.send(Json{{"type", "response.create"}, {"input", {message("second")}}});
  waitFor([&] {return observer.values().size() == 4;});
  EXPECT_EQ(observer.values().at(3).at("response"), completed("r2"));
  EXPECT_EQ(server.connections(), 1u);
  ASSERT_EQ(server.paths().size(), 1u);
  EXPECT_EQ(server.paths().at(0), "/v1/responses");
  EXPECT_EQ(server.authorizations().at(0), "Bearer loopback-fixture");
  EXPECT_EQ(transport.getStatus().dump().find("loopback-fixture"), std::string::npos);
}

TEST(WsTransport, ApiErrorsAreForwardedWithoutBecomingNetworkErrors)
{
  const Json event{{"type", "error"}, {"error", {{"code", "model_error"}}}};
  WsFixture server([&](auto & peer, const Json &, std::size_t) {peer.send(event);});
  Observer observer;
  ResponsesWsTransport transport(fastOptions());
  observer.connect(transport, server.url());
  waitFor([&] {return transport.connected();});
  transport.send(Json{{"type", "response.create"}});
  waitFor([&] {return observer.values().size() == 1;});
  EXPECT_EQ(observer.values().at(0), event);
  EXPECT_TRUE(transport.connected());
  EXPECT_TRUE(transport.getStatus().at("last_error").is_null());
}

TEST(WsTransport, InvalidJsonReconnectsWithoutReplayingBusinessFrames)
{
  WsFixture server([](auto & peer, const Json &, std::size_t index) {
      if (index == 0) {peer.raw("{", false);}
      else {peer.send(Json{{"type", "response.completed"}, {"response", completed("recovered")}});}
    });
  Observer observer;
  ResponsesWsTransport transport(fastOptions());
  observer.connect(transport, server.url());
  waitFor([&] {return transport.connected();});
  transport.send(Json{{"type", "response.create"}, {"input", "one"}});
  waitFor([&] {
    const auto status = transport.getStatus();
    return status.at("connected").get<bool>() && status.at("connection_id").get<int>() >= 2;
  });
  EXPECT_TRUE(observer.values().empty());
  EXPECT_EQ(transport.getStatus().at("last_error").at("stage"), "decode");
  EXPECT_EQ(server.requests().size(), 1u);
  transport.send(Json{{"type", "response.create"}, {"input", "two"}});
  waitFor([&] {return observer.values().size() == 1;});
  EXPECT_EQ(observer.values().at(0).at("response").at("id"), "recovered");
}

TEST(WsTransport, SendsDuringUnavailableConnectionAreNotQueued)
{
  WsServerOptions server_options;
  server_options.stall_handshake = true;
  WsFixture server([](auto & peer, const Json &, std::size_t) {
      peer.send(Json{{"type", "response.completed"}, {"response", completed("explicit")}});
    }, server_options);
  Observer observer;
  ResponsesWsTransport transport(fastOptions());
  observer.connect(transport, server.url());
  transport.send(Json{{"type", "response.create"}, {"input", "discarded"}});
  server.release(std::numeric_limits<std::size_t>::max());
  waitFor([&] {return transport.connected();});
  std::this_thread::sleep_for(30ms);
  EXPECT_TRUE(server.requests().empty());
  transport.send(Json{{"type", "response.create"}, {"input", "explicit"}});
  waitFor([&] {return observer.values().size() == 1;});
  EXPECT_EQ(server.requests().size(), 1u);
  EXPECT_EQ(server.requests().at(0).at("input"), "explicit");
}

TEST(WsTransport, HandshakeTimeoutAndCloseCancelNetworkWaits)
{
  WsServerOptions stalled;
  stalled.stall_handshake = true;
  WsFixture server([](auto &, const Json &, std::size_t) {}, stalled);
  Observer observer;
  auto options = fastOptions();
  options.connect_timeout = 60ms;
  ResponsesWsTransport transport(options);
  observer.connect(transport, server.url());
  waitFor([&] {return !transport.getStatus().at("last_error").is_null();});
  EXPECT_EQ(transport.getStatus().at("last_error").at("stage"), "websocket");
  const auto begin = std::chrono::steady_clock::now();
  transport.close();
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 500ms);
  EXPECT_EQ(transport.getStatus().at("state"), "closed");

  ResponsesWsTransport immediate(options);
  observer.connect(immediate, server.url());
  immediate.close();
  EXPECT_EQ(immediate.getStatus().at("state"), "closed");
}

TEST(WsTransport, IdlePingDetectsPeerThatStopsReading)
{
  WsFixture server([&](auto &, const Json &, std::size_t) {server.waitRelease(0);});
  Observer observer;
  auto options = fastOptions();
  options.idle_timeout = 80ms;
  ResponsesWsTransport transport(options);
  observer.connect(transport, server.url());
  waitFor([&] {return transport.connected();});
  transport.send(Json{{"type", "response.create"}});
  waitFor([&] {return server.requests().size() == 1;});
  waitFor([&] {return !transport.getStatus().at("last_error").is_null();});
  EXPECT_EQ(transport.getStatus().at("last_error").at("stage"), "read");
  EXPECT_TRUE(observer.values().empty());
  transport.close();
}

TEST(WsTransport, RejectedUpgradeReportsNetworkAuthenticationFailure)
{
  WsServerOptions rejected;
  rejected.reject_upgrade = true;
  WsFixture server([](auto &, const Json &, std::size_t) {}, rejected);
  Observer observer;
  ResponsesWsTransport transport(fastOptions());
  observer.connect(transport, server.url());
  waitFor([&] {return !transport.getStatus().at("last_error").is_null();});
  EXPECT_EQ(transport.getStatus().at("last_error").at("stage"), "websocket");
  EXPECT_FALSE(transport.connected());
  EXPECT_TRUE(observer.values().empty());
}

TEST(WsTransport, InterruptRotatesConnectionWithoutAddingNetworkErrorOrReplaying)
{
  WsFixture server([](auto & peer, const Json &, std::size_t index) {
      peer.send(Json{{"type", "response.completed"},
          {"response", completed("r" + std::to_string(index + 1))}});
    });
  Observer observer;
  ResponsesWsTransport transport(fastOptions());
  observer.connect(transport, server.url());
  waitFor([&] {return transport.connected();});
  transport.send(Json{{"type", "response.create"}});
  waitFor([&] {return observer.values().size() == 1;});
  transport.interrupt();
  waitFor([&] {
    return transport.connected() && transport.getStatus().at("connection_id").get<int>() >= 2;
  });
  EXPECT_TRUE(transport.getStatus().at("last_error").is_null());
  EXPECT_EQ(server.requests().size(), 1u);
  transport.send(Json{{"type", "response.create"}});
  waitFor([&] {return observer.values().size() == 2;});
}

TEST(WsTransport, TlsVerifiesSystemTrustAndServerHostname)
{
  FixtureFiles files;
  const auto tls_options = testCertificate(files);
  WsFixture server([](auto & peer, const Json &, std::size_t) {
      peer.send(Json{{"type", "response.completed"}, {"response", completed("tls")}});
    }, tls_options);
  {
    TestTrust trusted(tls_options.certificate);
    Observer observer;
    ResponsesWsTransport transport(fastOptions());
    observer.connect(transport, server.url("localhost"));
    waitFor([&] {return transport.connected();});
    transport.send(Json{{"type", "response.create"}});
    waitFor([&] {return observer.values().size() == 1;});
    EXPECT_EQ(observer.values().at(0).at("response").at("id"), "tls");
    transport.close();

    Observer mismatch_observer;
    ResponsesWsTransport mismatch(fastOptions());
    mismatch_observer.connect(mismatch, server.url("127.0.0.1"));
    waitFor([&] {return !mismatch.getStatus().at("last_error").is_null();});
    EXPECT_EQ(mismatch.getStatus().at("last_error").at("stage"), "tls");
    EXPECT_FALSE(mismatch.connected());
    mismatch.close();
  }
  Observer observer;
  ResponsesWsTransport untrusted(fastOptions());
  observer.connect(untrusted, server.url("localhost"));
  waitFor([&] {return !untrusted.getStatus().at("last_error").is_null();});
  EXPECT_EQ(untrusted.getStatus().at("last_error").at("stage"), "tls");
  EXPECT_FALSE(untrusted.connected());
}

}  // namespace vln_agent::test
