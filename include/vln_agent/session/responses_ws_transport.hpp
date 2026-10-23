#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "vln_agent/utils/json.hpp"

namespace vln_agent
{

/** @brief 仅约束网络阶段的超时和重连间隔，不包含模型生成超时。 */
struct WsTransportOptions
{
  std::chrono::milliseconds connect_timeout{10000};  ///< DNS 到 WS 握手的总时限。
  std::chrono::milliseconds write_timeout{10000};    ///< 单帧写入时限。
  std::chrono::milliseconds idle_timeout{60000};     ///< 空闲探活及失联检测时限。
  std::chrono::milliseconds reconnect_delay{250};    ///< 首次重连等待。
  std::chrono::milliseconds max_reconnect_delay{5000};  ///< 重连退避上限。
};

/** @brief 独立的持久 WS/WSS 通道；网络故障自动重连，业务帧不自动重发。 */
class ResponsesWsTransport
{
public:
  using EventHandler = std::function<void(const Json & event)>;
  using StateHandler = std::function<void(const Json & state)>;

  explicit ResponsesWsTransport(WsTransportOptions options = {});
  ~ResponsesWsTransport();

  /**
   * @brief 启动网络线程；认证只放在 Authorization 头，不写入状态快照。
   * @note 每个实例连接一次；回调运行在网络线程，须短时返回且不调用 close。
   */
  void connect(
    const std::string & base_url, const std::string & api_key,
    EventHandler on_event, StateHandler on_state);
  /** @brief 提交一帧；通道不可用或提交期间换代时丢弃该帧并通知状态，不排队重发。 */
  void send(Json event);
  /** @brief 当前是否已经完成 WS 握手。 */
  bool connected() const;
  /** @brief 返回连接状态、连接代号和最近网络错误，不含密钥。 */
  Json getStatus() const;
  /**
   * @brief 放弃当前连接并由底层重新连接，隔离已超时请求的迟到事件。
   * @note 不修改模型上下文、不补发请求，也不把主动中断记录为网络错误。
   */
  void interrupt();
  /** @brief 停止重连、取消网络等待并回收网络线程；可重复调用。 */
  void close();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vln_agent
