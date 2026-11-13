#include "vln_agent/plugins/tools/navigation/get_robot_state.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"
#include "vln_agent/utils/time.hpp"

#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace vln_agent
{
ToolResult NavigationRuntime::get_robot_state()
{
  auto result = ToolResult::success({{"motion", "idle"}, {"task", nullptr},
      {"tf", {{"available", false}, {"frame_id", map_frame_},
        {"child_frame_id", base_frame_}, {"age_sec", nullptr}}}, {"camera", nullptr}});
  auto & data = result.payload["data"];
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");}
    if (task_) {
      data["task"] = snapshot(task_).payload.at("data");
      data["task"]["action"] = task_->through_poses ? "navigate_through_poses" : "navigate_to_pose";
      data["motion"] = data["task"]["action"];
    }
    if (lookaround_running_) {data["motion"] = "lookaround";}
  }
  // 查询失败只影响 TF 字段，仍保留活动任务；不回退到可能陈旧的 Action 反馈位姿。
  try {
    const auto robot = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    result.payload["pose"] = utils::posePayload(map_frame_, robot.transform.translation.x,
      robot.transform.translation.y, tf2::getYaw(robot.transform.rotation));
    data["tf"]["available"] = true;
    const auto & stamp = robot.header.stamp;
    if (stamp.sec != 0 || stamp.nanosec != 0) {
      data["tf"]["age_sec"] =
        (node_.now() - rclcpp::Time(stamp, node_.get_clock()->get_clock_type())).seconds();
      bool simulated = false;
      node_.get_parameter_or("use_sim_time", simulated, false);
      if (!simulated) {
        result.payload["pose"]["time_stamp"] = utils::formatUtcTime(
          std::chrono::system_clock::time_point(std::chrono::seconds(stamp.sec) +
          std::chrono::nanoseconds(stamp.nanosec)));
      }
    }
  } catch (const tf2::TransformException & error) {
    data["tf"]["message"] = error.what();
  }
  return result;
}

void GetRobotState::initialize(const ToolContext & context, const Json & settings)
{
  resources_ = context.resources;
  rgbd_resource_ = settings.value("rgbd_resource", std::string());
  navigation_ = resources_->getOrCreate<NavigationRuntime>(context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult GetRobotState::execute(const Json &)
{
  auto result = navigation_->get_robot_state();
  if (result.payload.at("ok").get<bool>() && !rgbd_resource_.empty()) {
    result.payload["data"]["camera"] = resources_->get<RgbdRuntime>(rgbd_resource_)->camera_state();
  }
  return result;
}
}  // namespace vln_agent
PLUGINLIB_EXPORT_CLASS(vln_agent::GetRobotState, vln_agent::ToolPluginBase)
