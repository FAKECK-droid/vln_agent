#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"

#include <chrono>
#include <future>
#include <stdexcept>
#include "vln_agent/plugins/tools/rgbd/segmentation.hpp"

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
  segmenter_ = std::make_unique<rgbd::Segmenter>(settings.value("segmentation", Json::object()));
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

Json RgbdRuntime::camera_state()
{
  Json state{{"status", "unavailable"}, {"latest_pair_available", nullptr},
    {"latest_pair_age_sec", nullptr}};
  if (!client_->service_is_ready()) {return state;}
  auto request = std::make_shared<GetImage::Request>();
  request->operation = GetImage::Request::STATUS;
  auto future = client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(timeout_sec_)) != std::future_status::ready) {
    client_->remove_pending_request(future);
    state["status"] = "timeout";
    return state;
  }
  const auto response = future.get();
  if (!response->success) {
    state["status"] = "error";
    state["message"] = response->message;
    return state;
  }
  state["status"] = response->latest_available ? "ready" : "no_pair";
  state["latest_pair_available"] = response->latest_available;
  if (response->latest_available) {state["latest_pair_age_sec"] = response->latest_age_sec;}
  return state;
}

void RgbdRuntime::shutdown() noexcept
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return;}
    closed_ = true;
    condition_.notify_all();
  }
  if (callbacks_) {callbacks_->shutdown();}
  if (client_) {client_->prune_pending_requests();}
  // 先停止标定回调，再释放缓存，防止关闭期间的最后一条回调重新写入标定。
  std::lock_guard<std::mutex> lock(mutex_);
  pairs_.clear();
  surround_.reset();
  calibration_.reset();
  info_.reset();
}
}  // namespace vln_agent
