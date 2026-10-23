#pragma once

#include <memory>
#include <string>

#include "vln_agent/utils/json.hpp"

namespace vln_agent
{

/** @brief 独立模型上下文链，串行发送、两份输入缓冲及单个完整结果槽。 */
class SessionManager
{
public:
  /** @brief 读取全局 YAML 的 session_manager 段并启动连接，不发送初始化请求。 */
  explicit SessionManager(const std::string & global_yaml_path);
  ~SessionManager();

  /** Agent请求调用函数 */
  /** @brief 替换下次发送使用的原生 tools 数组，不触发发送。 */
  void setTools(const Json & tools);
  /** @brief 追加一个原生 API 输入对象，或按元素追加 JSON 数组。 */
  void addInputs(const Json & input);
  /** @brief 按顺序整批追加输入，仅获取一次会话锁。 */
  void addInputs(const Json::array_t & input_items);
  /** @brief 合并登记发送意图；错误或中断后的重试须显式调用本方法。 */
  void request();
  /** @brief 非阻塞取走原始完整 response，尚未完成时返回 null。 */
  Json takeResponse();

  /** 工具、状态、请求、输入查讯专用函数 */
  /** @brief 返回下次发送将使用的工具定义副本。 */
  Json getTools() const;
  /** @brief 返回尚未发送的原生输入数组副本；查询不消费输入。 */
  Json getPendingInputs() const;
  /** @brief 返回在途或失败后保留的完整请求副本，没有当前请求时返回 null。 */
  Json getCurrentRequest() const;
  /**
   * @brief 返回请求状态、响应 ID、重试标志、缓冲计数、会话错误及连接快照。
   * @note request_elapsed_ms 从本轮发送计时，结束后冻结；rate_limit_remaining_ms 向上取整。
   * @note can_retry 表示允许调用 request 登记重试，实际发送仍需等待连接和限频。
   */
  Json getRequestStatus() const;
  /** @brief 返回最新完成链头、预算、最近 usage 和已观察到的压缩次数。 */
  Json getContextStatus() const;
  /** @brief 返回本轮显示文本快照；关闭显示时返回 null，不消费完整结果。 */
  Json getStreamSnapshot() const;
  /** @brief 停止调度、关闭传输并回收内部线程；可重复调用。 */
  void shutdown();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vln_agent
