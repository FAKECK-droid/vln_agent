#include "vln_agent/plugins/tools/rgbd/depth_project.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"
#include "vln_agent/plugins/tools/rgbd/segmentation.hpp"
#include <algorithm>
#include <stdexcept>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace vln_agent
{
namespace
{
rgbd::InstancePrompt parsePrompt(const Json & item, cv::Size size, bool segmented)
{
  rgbd::InstancePrompt prompt;
  const auto & points = item.at("points");
  if (points.size() > 63) {throw std::invalid_argument("每个实例最多 63 个提示点");}
  prompt.points.reserve(points.size());
  for (const auto & point : points) {
    const int x = point.at("x").get<int>(), y = point.at("y").get<int>();
    const int label = point.at("label").get<int>();
    if (x < 0 || y < 0 || x >= size.width || y >= size.height || (label != 0 && label != 1)) {
      throw std::invalid_argument("提示点须位于返回图片内，label 为 0 或 1");
    }
    prompt.points.push_back({x, y, label});
  }
  if (segmented && !item.at("box").is_null()) {
    const auto & box = item.at("box");
    const int x0 = box.at("x_min").get<int>(), y0 = box.at("y_min").get<int>();
    const int x1 = box.at("x_max").get<int>(), y1 = box.at("y_max").get<int>();
    if (x0 < 0 || y0 < 0 || x1 > size.width || y1 > size.height || x1 <= x0 || y1 <= y0 ||
      points.size() > 62)
    {
      throw std::invalid_argument("框须位于返回图片边界内且有面积，带框最多 62 点");
    }
    prompt.box = cv::Rect2f(float(x0), float(y0), float(x1 - x0), float(y1 - y0));
  }
  return prompt;
}
}

ToolResult RgbdRuntime::depth_project(const Json & arguments)
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
  return projectDepth(pair, arguments);
}

ToolResult RgbdRuntime::projectDepth(const std::shared_ptr<Pair> & pair, const Json & arguments)
{
  const auto & id = pair->id;
  const auto target = arguments.at("target_frame").get<std::string>();
  const bool segmented = arguments.at("use_segmentation").get<bool>();
  const auto & instances = arguments.at("instances");
  if (instances.empty()) {return ToolResult::failure("INVALID_ARGUMENTS", "instances 至少包含一个实例");}
  try {
    std::call_once(pair->decode_once, [&] {
      auto depth = rgbd::decodeDepth(pair->images->depth);
      if (depth.pixels.cols != static_cast<int>(pair->info->width) ||
        depth.pixels.rows != static_cast<int>(pair->info->height))
      {
        throw std::invalid_argument("对齐深度分辨率必须与彩色 CameraInfo 一致");
      }
      pair->depth = std::move(depth);
    });
  } catch (const std::exception & error) {
    return ToolResult::failure("DEPTH_UNAVAILABLE", error.what());
  }
  tf2::Transform transform;
  tf2::fromMsg(pair->fixed_from_camera.transform, transform);
  if (target != fixed_frame_) {
    try {
      const auto current = tf_buffer_->lookupTransform(
        target, fixed_frame_, tf2::TimePointZero, tf2::durationFromSec(timeout_sec_));
      tf2::Transform target_from_fixed;
      tf2::fromMsg(current.transform, target_from_fixed);
      transform = target_from_fixed * transform;
    } catch (const tf2::TransformException & error) {
      return ToolResult::failure("TF_UNAVAILABLE", error.what());
    }
  }
  std::unique_lock<std::mutex> segmentation_lock(segmentation_mutex_, std::defer_lock);
  if (segmented) {
    segmentation_lock.lock();
    try {
      segmenter_->setImage(pair->id, pair->color);
      std::call_once(pair->calibration->rays_once, [&] {
        pair->calibration->rays = rgbd::projectionRays(*pair->info);
      });
    } catch (const std::exception & error) {
      return ToolResult::failure("SEGMENTATION_UNAVAILABLE", error.what());
    }
  }
  Json rows = Json::array();
  rows.get_ref<Json::array_t &>().reserve(instances.size());
  std::size_t successes = 0;
  for (const auto & instance : instances) {
    Json row{{"index", rows.size()}, {"ok", false}};
    try {
      const auto prompt = parsePrompt(instance, pair->image_size, segmented);
      rgbd::DepthMean mean;
      if (segmented) {
        auto mask = segmenter_->predict(prompt, pair->image_size);
        row["mask_score"] = mask.score;
        const cv::Mat pixels(mask.size, CV_8UC1, mask.pixels.data());
        mean = rgbd::meanMaskedDepth(pair->depth, pair->calibration->rays, pixels);
      } else {
        std::vector<cv::Point> points;
        points.reserve(prompt.points.size());
        for (const auto & point : prompt.points) {
          if (point.label == 1) {points.emplace_back(point.x, point.y);}
        }
        mean = rgbd::meanPointDepth(pair->depth, *pair->info, points, pair->image_size);
      }
      row["sample_count"] = mean.count;
      if (mean.count) {
        // 刚性变换保持均值；只变换最终三维均值，省去逐像素 TF 和点云。
        const auto point = transform * tf2::Vector3(mean.point[0], mean.point[1], mean.point[2]);
        row["ok"] = true;
        row["x"] = point.x(); row["y"] = point.y(); row["z"] = point.z();
        row["depth_m"] = mean.point[2];
        ++successes;
      } else {
        row["code"] = "DEPTH_UNAVAILABLE";
        row["message"] = segmented ? "实例掩码内没有有效深度" : "正类提示点没有有效深度";
      }
    } catch (const std::invalid_argument & error) {
      row["code"] = "INVALID_PROMPT"; row["message"] = error.what();
    } catch (const std::exception & error) {
      row["code"] = "PROJECTION_FAILED"; row["message"] = error.what();
    }
    rows.push_back(std::move(row));
  }
  auto result = successes ? ToolResult::success(Json::object()) :
    ToolResult::failure(rows.front().at("code").get<std::string>(),
      rows.front().at("message").get<std::string>());
  result.payload["data"] = {{"pair_id", id}, {"frame_id", target},
    {"use_segmentation", segmented}, {"success_count", successes}, {"instances", std::move(rows)}};
  return result;
}

void DepthProject::initialize(const ToolContext & context, const Json &)
{
  rgbd_ = context.resources->getOrCreate<RgbdRuntime>(context.group_id + "/rgbd", [&] {
      return std::make_shared<RgbdRuntime>(context.node, context.group_settings);
    });
}
ToolResult DepthProject::execute(const Json & arguments)
{
  return rgbd_->depth_project(arguments);
}
}  // namespace vln_agent
PLUGINLIB_EXPORT_CLASS(vln_agent::DepthProject, vln_agent::ToolPluginBase)
