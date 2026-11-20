#pragma once

#include <functional>
#include <string>

#include "vln_agent/utils/json.hpp"

namespace vln_agent
{
/** @brief 单请求 Responses 通道。事件回调短时返回，连接状态中的 connected 表示可接收请求。 */
class ResponsesTransport
{
public:
  using EventHandler = std::function<void(const Json &)>;
  using StateHandler = std::function<void(const Json &)>;

  virtual ~ResponsesTransport() = default;
  virtual void connect(
    const std::string & base_url, const std::string & api_key,
    EventHandler on_event, StateHandler on_state) = 0;
  /** @brief 发送 Responses 请求体；传输插件负责自身协议包装。 */
  virtual void send(Json request) = 0;
  virtual Json getStatus() const = 0;
  /** @brief 丢弃在途请求和迟到事件；恢复可用后通知会话，业务不自动重发。 */
  virtual void interrupt() = 0;
  /** @brief 停止网络处理，返回前回收线程。 */
  virtual void close() = 0;
};
}  // namespace vln_agent
