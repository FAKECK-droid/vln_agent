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
  return getDepthPoints(arguments.at("image_pair_id").get<std::string>(),
    {{arguments.at("pixel_x").get<int>(), arguments.at("pixel_y").get<int>()}},
    arguments.at("target_frame").get<std::string>()).front();
}

std::vector<ToolResult> RgbdRuntime::getDepthPoints(
  const std::string & id, const std::vector<cv::Point> & pixels, const std::string & target)
{
  const auto failed = [&](const std::string & code, const std::string & message) {
      return std::vector<ToolResult>(pixels.size(), ToolResult::failure(code, message));
    };
  std::shared_ptr<Pair> pair;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = std::find_if(pairs_.begin(), pairs_.end(),
      [&](const auto & item) {return item->id == id;});
    if (found == pairs_.end()) {
      return failed("IMAGE_PAIR_NOT_FOUND", "图像对已淘汰或不存在: " + id);
    }
    pair = *found;
  }
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
  } catch (const std::exception & error) {
    return failed("DEPTH_UNAVAILABLE", error.what());
  }
  geometry_msgs::msg::TransformStamped target_from_fixed;
  if (target != fixed_frame_) {
    try {
      target_from_fixed = tf_buffer_->lookupTransform(
        target, fixed_frame_, tf2::TimePointZero, tf2::durationFromSec(timeout_sec_));
    } catch (const tf2::TransformException & error) {
      return failed("TF_UNAVAILABLE", error.what());
    }
  }
  std::vector<ToolResult> results;
  results.reserve(pixels.size());
  for (const auto & input : pixels) {
    try {
      const auto pixel = rgbd::toDepthPixel(
        input.x, input.y, pair->image_size, pair->depth.pixels.size());
      const double depth = rgbd::depthAt(pair->depth, pixel.x, pixel.y);
      geometry_msgs::msg::PointStamped point;
      point.point = rgbd::backProject(*pair->info, pixel.x, pixel.y, depth);
      tf2::doTransform(point, point, pair->fixed_from_camera);
      if (target != fixed_frame_) {tf2::doTransform(point, point, target_from_fixed);}
      results.push_back(ToolResult::success({
        {"image_pair_id", id}, {"pixel_x", input.x}, {"pixel_y", input.y},
        {"depth_pixel_x", pixel.x}, {"depth_pixel_y", pixel.y},
        {"depth_m", depth}, {"frame_id", target},
        {"x", point.point.x}, {"y", point.point.y}, {"z", point.point.z}}));
    } catch (const std::exception & error) {
      results.push_back(ToolResult::failure("DEPTH_UNAVAILABLE", error.what()));
    }
  }
  return results;
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
