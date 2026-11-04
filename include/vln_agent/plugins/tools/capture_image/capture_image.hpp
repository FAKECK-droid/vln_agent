#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rgbd_image_queue/srv/get_image.hpp>

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{

class ToolCallbacks;

/** @brief 通过 GetImage 服务读取彩色压缩图，直接返回原生图片内容项。 */
class CaptureImage : public ToolPluginBase
{
public:
  ~CaptureImage() override;

  /** @brief 从组配置读取 service_name，从函数配置读取 timeout_sec。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief mode 为 latest/index/age，detail 为历史索引或毫秒数。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 停止服务回调，释放挂起请求并唤醒等待响应的调用。 */
  void shutdown() noexcept override;

private:
  using GetImage = rgbd_image_queue::srv::GetImage;

  bool real_stamps_{true};  ///< 仿真时间不伪装成 UTC 采样时间。
  std::unique_ptr<ToolCallbacks> callbacks_;  ///< 独占服务回调组及其执行线程。
  rclcpp::Client<GetImage>::SharedPtr client_;
  double timeout_sec_{2.0};  ///< 服务发现和响应等待各自的时限。
};

}  // namespace vln_agent
