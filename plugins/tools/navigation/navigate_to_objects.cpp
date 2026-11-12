#include "vln_agent/plugins/tools/navigation/navigate_to_objects.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"

#include <cmath>
#include <vector>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace vln_agent
{
ToolResult NavigationRuntime::navigate_to_objects(RgbdRuntime & rgbd, const Json & arguments)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");}
    // 活动任务可直接返回，省去分割；提交 Action 时沿用现有任务槽的最终互斥判断。
    if (task_) {return ToolResult::failure("NAVIGATION_BUSY", "已有活动导航任务");}
  }
  const auto & instances = arguments.at("instances");
  std::vector<double> distances;
  distances.reserve(instances.size());
  for (const auto & instance : instances) {
    const auto distance = instance.value("stand_off_m", Json(0));
    const double value = distance.is_null() ? 0.0 : distance.get<double>();
    if (!std::isfinite(value) || value < 0) {
      return ToolResult::failure("INVALID_ARGUMENTS", "stand_off_m 必须为有限非负数");
    }
    distances.push_back(value);
  }

  Json projection_input = arguments;
  projection_input["target_frame"] = map_frame_;
  auto projected = rgbd.depth_project(projection_input);
  if (!projected.payload.at("ok").get<bool>()) {return projected;}
  const auto & objects = projected.payload.at("data").at("instances");
  if (projected.payload.at("data").at("success_count").get<std::size_t>() != instances.size()) {
    projected.payload["ok"] = false;
    projected.payload["code"] = "INSTANCE_PROJECTION_FAILED";
    projected.payload["message"] = "存在投影失败的实例，本次导航未启动，明细见 data.instances";
    return projected;
  }

  navigation::Pose2D previous;
  try {
    const auto pose = tf_buffer_->lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(costmap_timeout_sec_));
    previous = {pose.transform.translation.x, pose.transform.translation.y,
      tf2::getYaw(pose.transform.rotation)};
  } catch (const tf2::TransformException & error) {
    auto failed = ToolResult::failure("TF_UNAVAILABLE", error.what());
    failed.payload["data"]["projection"] = std::move(projected.payload["data"]);
    return failed;
  }

  std::vector<navigation::Pose2D> goals;
  goals.reserve(objects.size());
  for (std::size_t i = 0; i < objects.size(); ++i) {
    const double x = objects.at(i).at("x").get<double>();
    const double y = objects.at(i).at("y").get<double>();
    const double dx = x - previous.x, dy = y - previous.y;
    const double approach = dx != 0 || dy != 0 ? std::atan2(dy, dx) : previous.yaw;
    navigation::Pose2D goal{x, y, approach};
    if (distances[i] > 0) {
      goal.x -= distances[i] * std::cos(approach);
      goal.y -= distances[i] * std::sin(approach);
    }
    goals.push_back(goal);
    previous = goal;
  }
  // 中间点朝向下一个停靠点；最后一点保留面向物体的接近方向。
  // 零停靠距离使终点与物体重合，此时同样使用接近方向。
  for (std::size_t i = 0; i + 1 < goals.size(); ++i) {
    const double dx = goals[i + 1].x - goals[i].x, dy = goals[i + 1].y - goals[i].y;
    if (dx != 0 || dy != 0) {goals[i].yaw = std::atan2(dy, dx);}
  }
  Json poses = Json::array();
  poses.get_ref<Json::array_t &>().reserve(goals.size());
  for (const auto & goal : goals) {
    poses.push_back({{"x", goal.x}, {"y", goal.y}, {"yaw", goal.yaw}});
  }
  const bool multiple = poses.size() > 1;
  auto started = multiple ?
    start_through_poses_navigation({{"frame_id", map_frame_}, {"poses", poses}}) :
    start_navigation({{"frame_id", map_frame_}, {"x", poses.front().at("x")},
        {"y", poses.front().at("y")}, {"yaw", poses.front().at("yaw")}});
  auto & data = started.payload["data"];
  data["action"] = multiple ? "navigate_through_poses" : "navigate_to_pose";
  data["frame_id"] = map_frame_;
  data["poses"] = std::move(poses);
  data["projection"] = std::move(projected.payload["data"]);
  return started;
}

void NavigateToObjects::initialize(const ToolContext & context, const Json & settings)
{
  resources_ = context.resources;
  rgbd_resource_ = settings.value("rgbd_resource", std::string("rgbd/rgbd"));
  navigation_ = resources_->getOrCreate<NavigationRuntime>(context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult NavigateToObjects::execute(const Json & arguments)
{
  const auto rgbd = resources_->get<RgbdRuntime>(rgbd_resource_);
  return navigation_->navigate_to_objects(*rgbd, arguments);
}
}  // namespace vln_agent
PLUGINLIB_EXPORT_CLASS(vln_agent::NavigateToObjects, vln_agent::ToolPluginBase)
