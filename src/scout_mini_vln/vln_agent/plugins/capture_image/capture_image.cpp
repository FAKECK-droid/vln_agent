#include "capture_image.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "vln_agent/tools/ros_callbacks.hpp"
#include "vln_agent/utils/base64.hpp"

namespace vln_agent
{
CaptureImage::~CaptureImage()
{
  shutdown();
}

void CaptureImage::initialize(const ToolContext & context, const Json & settings)
{
  timeout_sec_ = settings.value("timeout_sec", 2.0);
  if (timeout_sec_ <= 0) {
    throw std::invalid_argument("相机 timeout_sec 必须大于零");
  }
  callbacks_ = std::make_unique<ToolCallbacks>(context.node);
  state_ = std::make_shared<State>();
  rclcpp::SubscriptionOptions options;
  options.callback_group = callbacks_->group();
  subscription_ = context.node.create_subscription<sensor_msgs::msg::Image>(
    context.group_settings.at("camera_topic").get<std::string>(), rclcpp::SensorDataQoS(),
    [state = state_](sensor_msgs::msg::Image::ConstSharedPtr image) {
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->latest = std::move(image);
      }
      state->condition.notify_all();
    }, options);
}

ToolResult CaptureImage::execute(const Json &)
{
  sensor_msgs::msg::Image::ConstSharedPtr image;
  {
    std::unique_lock<std::mutex> lock(state_->mutex);
    if (!state_->condition.wait_for(lock, std::chrono::duration<double>(timeout_sec_),
        [this] {return state_->closed || state_->latest != nullptr;}))
    {
      return ToolResult::failure("IMAGE_TIMEOUT", "等待相机首帧超时");
    }
    if (state_->closed) {
      return ToolResult::failure("TOOL_MANAGER_CLOSED", "相机工具已经关闭");
    }
    image = state_->latest;
  }
  const auto frame = cv_bridge::toCvShare(image, "bgr8");
  std::vector<unsigned char> jpeg;
  if (!cv::imencode(".jpg", frame->image, jpeg)) {
    return ToolResult::failure("IMAGE_ENCODING_FAILED", "无法将图像编码为 JPEG");
  }
  return {
    {{"ok", true}, {"frame_id", image->header.frame_id},
      {"stamp", {{"sec", image->header.stamp.sec}, {"nanosec", image->header.stamp.nanosec}}},
      {"width", image->width}, {"height", image->height}},
    {{{"type", "input_image"}, {"image_url", "data:image/jpeg;base64," + utils::encodeBase64(jpeg)},
      {"detail", "auto"}}}};
}

void CaptureImage::shutdown() noexcept
{
  if (!state_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->closed) {
      return;
    }
    state_->closed = true;
  }
  state_->condition.notify_all();
  callbacks_->shutdown();
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::CaptureImage, vln_agent::ToolPluginBase)

