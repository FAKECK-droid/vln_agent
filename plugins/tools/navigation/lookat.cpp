#include "vln_agent/plugins/tools/navigation/lookat.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace vln_agent
{
ToolResult RgbdRuntime::imageTurn(const Json & arguments, double & radians)
{
  const auto id = arguments.at("pair_id").get<std::string>();
  std::shared_ptr<Pair> pair;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = std::find_if(pairs_.begin(), pairs_.end(),
      [&](const auto & value) {return value->id == id;});
    if (found == pairs_.end()) {
      return ToolResult::failure("IMAGE_PAIR_NOT_FOUND", "图像对已淘汰或不存在: " + id);
    }
    pair = *found;
  }

  const double x = arguments.at("pixel_x").get<double>();
  const double y = arguments.at("pixel_y").get<double>();
  if (!(x >= 0 && x <= pair->image_size.width - 1 && y >= 0 && y <= pair->image_size.height - 1)) {
    return ToolResult::failure("INVALID_PIXEL", "像素点须位于返回的彩色图内");
  }
  const auto & info = *pair->info;
  const auto ray = rgbd::pixelRay(info,
    (x + 0.5) * info.width / pair->image_size.width - 0.5,
    (y + 0.5) * info.height / pair->image_size.height - 0.5);

  // 优先使用彩色图采样时的朝向；若 TF 已过期，使用配对时保存的相机朝向。
  auto sampled_rotation = pair->fixed_from_camera.transform.rotation;
  const auto & stamp = pair->images->color.header.stamp;
  if (stamp.sec != 0 || stamp.nanosec != 0) {
    try {
      sampled_rotation = tf_buffer_->lookupTransform(
        fixed_frame_, info.header.frame_id, rclcpp::Time(stamp)).transform.rotation;
    } catch (const tf2::TransformException &) {}
  }
  tf2::Quaternion sampled, current;
  tf2::fromMsg(sampled_rotation, sampled);
  try {
    tf2::fromMsg(tf_buffer_->lookupTransform(
      fixed_frame_, info.header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(timeout_sec_)).transform.rotation, current);
  } catch (const tf2::TransformException & error) {
    return ToolResult::failure("TF_UNAVAILABLE", error.what());
  }

  const auto target = tf2::quatRotate(sampled, tf2::Vector3(ray.x, ray.y, 1.0));
  const auto forward = tf2::quatRotate(current, tf2::Vector3(0, 0, 1));
  if (!(std::hypot(target.x(), target.y()) > 1e-6 &&
    std::hypot(forward.x(), forward.y()) > 1e-6))
  {
    return ToolResult::failure("INVALID_BEARING", "图像点或相机光轴没有可用的水平方位");
  }
  radians = std::remainder(std::atan2(target.y(), target.x()) -
      std::atan2(forward.y(), forward.x()), 2 * std::acos(-1.0));
  return ToolResult::success({{"pair_id", id}});
}

ToolResult NavigationRuntime::lookat(RgbdRuntime & rgbd, const Json & arguments)
{
  return withRelativeMotion([&] {
      double radians = 0;
      auto bearing = rgbd.imageTurn(arguments, radians);
      if (!bearing.payload.at("ok").get<bool>()) {return bearing;}
      auto result = std::abs(radians) > 1e-6 ?
        spinRelative(radians) : ToolResult::success({{"status", "succeeded"}});
      result.payload["data"]["pair_id"] = std::move(bearing.payload.at("data").at("pair_id"));
      result.payload["data"]["yaw_deg"] = radians * 180.0 / std::acos(-1.0);
      return result;
    });
}

void Lookat::initialize(const ToolContext & context, const Json & settings)
{
  resources_ = context.resources;
  rgbd_resource_ = settings.value("rgbd_resource", std::string("rgbd/rgbd"));
  navigation_ = resources_->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult Lookat::execute(const Json & arguments)
{
  return navigation_->lookat(*resources_->get<RgbdRuntime>(rgbd_resource_), arguments);
}
}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::Lookat, vln_agent::ToolPluginBase)
