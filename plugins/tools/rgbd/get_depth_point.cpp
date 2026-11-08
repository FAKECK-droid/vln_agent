#include "vln_agent/plugins/tools/rgbd/get_depth_point.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"

#include <algorithm>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace vln_agent
{
ToolResult RgbdRuntime::get_depth_point(const Json & arguments)
{
  const auto id = arguments.at("image_pair_id").get<std::string>();
  const int u = arguments.at("pixel_x").get<int>(), v = arguments.at("pixel_y").get<int>();
  const auto target = arguments.at("target_frame").get<std::string>();
  std::shared_ptr<Pair> pair;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = std::find_if(pairs_.begin(), pairs_.end(),
      [&](const auto & item) {return item->id == id;});
    if (found == pairs_.end()) {
      return ToolResult::failure("IMAGE_PAIR_NOT_FOUND", "图像对已淘汰或不存在: " + id);
    }
    pair = *found;
  }
  double depth;
  geometry_msgs::msg::PointStamped point;
  try {
    std::call_once(pair->decode_once, [&] {
      auto decoded = rgbd::decodeDepth(pair->images->depth);
      if (decoded.pixels.cols != static_cast<int>(pair->info->width) ||
        decoded.pixels.rows != static_cast<int>(pair->info->height))
      {
        throw std::invalid_argument("对齐深度分辨率必须与彩色 CameraInfo 一致");
      }
      pair->depth = std::move(decoded);
    });
    depth = rgbd::depthAt(pair->depth, u, v);
    point.point = rgbd::backProject(*pair->info, u, v, depth);
  } catch (const std::exception & error) {
    return ToolResult::failure("DEPTH_UNAVAILABLE", error.what());
  }
  tf2::doTransform(point, point, pair->fixed_from_camera);
  if (target != fixed_frame_) {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        target, fixed_frame_, tf2::TimePointZero, tf2::durationFromSec(timeout_sec_));
      tf2::doTransform(point, point, transform);
    } catch (const tf2::TransformException & error) {
      return ToolResult::failure("TF_UNAVAILABLE", error.what());
    }
  }
  return ToolResult::success({{"image_pair_id", id}, {"pixel_x", u}, {"pixel_y", v},
    {"depth_m", depth}, {"frame_id", target},
    {"x", point.point.x}, {"y", point.point.y}, {"z", point.point.z}});
}

void GetDepthPoint::initialize(const ToolContext & context, const Json &)
{
  rgbd_ = context.resources->getOrCreate<RgbdRuntime>(
    context.group_id + "/rgbd", [&] {
      return std::make_shared<RgbdRuntime>(context.node, context.group_settings);
    });
}

ToolResult GetDepthPoint::execute(const Json & arguments)
{
  return rgbd_->get_depth_point(arguments);
}
}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::GetDepthPoint, vln_agent::ToolPluginBase)
