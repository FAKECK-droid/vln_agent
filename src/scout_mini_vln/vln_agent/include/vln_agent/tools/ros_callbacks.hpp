#pragma once

#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace vln_agent
{

/**
 * @brief 为插件的独占回调组提供可回收的执行线程，复用宿主节点。
 * @note 回调应短时返回；只从回调之外调用 shutdown，返回后该组不再执行回调。
 */
class ToolCallbacks
{
public:
  /** @brief 创建不自动加入宿主执行器的互斥回调组，并启动执行线程。 */
  explicit ToolCallbacks(rclcpp::Node & node);
  ~ToolCallbacks();

  /** @brief 供插件创建订阅和 Action 客户端时指定回调组。 */
  const rclcpp::CallbackGroup::SharedPtr & group() const {return group_;}

  /** @brief 停止调度并等待当前回调及执行线程退出；可重复调用。 */
  void shutdown() noexcept;

private:
  rclcpp::Context::SharedPtr context_;       ///< 使用宿主节点的 ROS 上下文。
  rclcpp::CallbackGroup::SharedPtr group_;  ///< 插件独占互斥回调组。
  rclcpp::executors::SingleThreadedExecutor executor_;  ///< 只执行当前组的回调。
  std::atomic_bool closed_{false};          ///< 执行循环关闭标志。
  std::thread thread_;                     ///< 在资源卸载前回收的线程。
};

}  // namespace vln_agent
