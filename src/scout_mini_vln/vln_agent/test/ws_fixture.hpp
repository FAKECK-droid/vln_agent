#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "vln_agent/utils/json.hpp"

namespace vln_agent::test
{
using namespace std::chrono_literals;

/** @brief 保存测试配置与证书到构建目录，保留现场便于复现。 */
class FixtureFiles
{
public:
  FixtureFiles();
  std::string write(const std::string & name, const std::string & text) const;
  std::string sessionConfig(const std::string & url, const Json & overrides = Json::object()) const;
  std::string directory;
};

/** @brief 回环服务端的 TLS 和握手故障注入配置。 */
struct WsServerOptions
{
  bool tls{false};
  bool reject_upgrade{false};
  bool stall_handshake{false};
  std::string certificate, private_key;
};

/** @brief 同步服务端只在自己的线程发送，外部测试通过门闩控制回复时机。 */
class WsFixture
{
public:
  struct Peer
  {
    std::function<void(const std::string &, bool)> raw;
    std::function<void()> drop;
    void send(const Json & event, bool fragment = false) {raw(event.dump(), fragment);}
  };
  using Handler = std::function<void(Peer &, const Json &, std::size_t)>;

  explicit WsFixture(Handler handler, WsServerOptions options = {});
  ~WsFixture();
  std::string url(const std::string & host = "127.0.0.1") const;
  std::vector<Json> requests() const;
  std::vector<std::chrono::steady_clock::time_point> starts() const;
  std::vector<std::string> paths() const;
  std::vector<std::string> authorizations() const;
  std::size_t connections() const;
  void release(std::size_t index);
  void waitRelease(std::size_t index);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/** @brief 构造保持原生消息结构的测试输入。 */
Json message(const std::string & text, const std::string & role = "user");
/** @brief 构造含完整原始输出和 usage 的测试 response。 */
Json completed(const std::string & id, const std::string & text = "完成");
/** @brief 最多等待三秒的协议测试观测，不运行任何真实模型。 */
void waitFor(const std::function<bool()> & predicate);
/** @brief 创建只用于回环 TLS 验证的短期自签名 localhost 证书。 */
WsServerOptions testCertificate(const FixtureFiles & files);

}  // namespace vln_agent::test
