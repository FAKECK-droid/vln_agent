#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>

#include <sensor_msgs/msg/image.hpp>

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{

class ToolCallbacks;

/** @brief 读取最新相机图像，并返回 JPEG 原生图片内容项。 */
class CaptureImage : public ToolPluginBase
{
public:
  ~CaptureImage() override;

  /** @brief 从组配置读取 camera_topic，从函数配置读取 timeout_sec。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 无参数调用；等待首帧，后续调用直接使用最新帧。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 唤醒首帧等待并停止相机回调；可重复调用。 */
  void shutdown() noexcept override;

private:
  struct State
  {
    std::mutex mutex;                      ///< 保护最近帧与关闭标志。
    std::condition_variable condition;     ///< 首帧及关闭通知。
    sensor_msgs::msg::Image::ConstSharedPtr latest;  ///< 最近收到的原始帧。
    bool closed{false};                    ///< 相机工具是否已关闭。
  };
  std::shared_ptr<State> state_;            ///< 回调和并发执行共用的状态。
  std::unique_ptr<ToolCallbacks> callbacks_;  ///< 独占回调组及其执行线程。
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;  ///< 相机订阅。
  double timeout_sec_{2.0};                 ///< 无首帧时的等待上限。
};

}  // namespace vln_agent
