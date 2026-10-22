#pragma once

#include <string>

namespace vln_agent::utils
{
/** @brief 已解析的 WebSocket 连接地址，主机头保留显式端口和 IPv6 方括号。 */
struct WebSocketUrl
{
  bool secure;                             ///< 是否使用 TLS。
  std::string host;                         ///< DNS/TLS 校验使用的主机。
  std::string host_header;                  ///< HTTP Upgrade 使用的主机头。
  std::string port;                         ///< 显式或协议默认端口。
  std::string target;                       ///< 标准化后的 Responses 路径。
};
/** @brief 将 http/https/ws/wss 基地址解析为 WS 地址；保留已有 /responses 后缀。 */
WebSocketUrl parseWebSocketUrl(const std::string & base_url);
}  // namespace vln_agent::utils
