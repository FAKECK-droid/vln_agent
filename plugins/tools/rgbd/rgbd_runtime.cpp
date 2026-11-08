#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"

#include <stdexcept>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "vln_agent/tools/ros_callbacks.hpp"

namespace vln_agent
{
RgbdRuntime::RgbdRuntime(rclcpp::Node & node, const Json & settings)
: node_(node)
{
  fixed_frame_ = settings.value("fixed_frame", std::string("map"));
  timeout_sec_ = settings.value("timeout_sec", 2.0);
  max_pairs_ = settings.value("max_pairs", std::size_t{8});
  if (max_pairs_ == 0) {throw std::invalid_argument("max_pairs 必须大于零");}
  bool simulated = false;
  node_.get_parameter_or("use_sim_time", simulated, false);
  real_stamps_ = !simulated;
  callbacks_ = std::make_unique<ToolCallbacks>(node_);
  client_ = node_.create_client<GetImage>(
    settings.at("service_name").get<std::string>(),
    rmw_qos_profile_services_default, callbacks_->group());
  rclcpp::SubscriptionOptions options;
  options.callback_group = callbacks_->group();
  info_subscription_ = node_.create_subscription<sensor_msgs::msg::CameraInfo>(
    settings.value("camera_info_topic", std::string("/camera/color/camera_info")),
    rclcpp::SensorDataQoS().keep_last(1),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
      std::lock_guard<std::mutex> lock(mutex_);
      info_ = std::move(info);
    }, options);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
  // TF 由本资源的回调线程填充，工具执行线程可带时限等待历史变换。
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
    *tf_buffer_, &node_, false, tf2_ros::DynamicListenerQoS(),
    tf2_ros::StaticListenerQoS(), options, options);
}

RgbdRuntime::~RgbdRuntime()
{
  shutdown();
}

void RgbdRuntime::shutdown() noexcept
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return;}
    closed_ = true;
  }
  if (callbacks_) {callbacks_->shutdown();}
  if (client_) {client_->prune_pending_requests();}
  // 先停止标定回调，再释放缓存，防止关闭期间的最后一条回调重新写入标定。
  std::lock_guard<std::mutex> lock(mutex_);
  pairs_.clear();
  info_.reset();
}
}  // namespace vln_agent
