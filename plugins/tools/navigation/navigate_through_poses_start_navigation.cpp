#include "vln_agent/plugins/tools/navigation/navigate_through_poses_start_navigation.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{

ToolResult NavigationRuntime::start_through_poses_navigation(const Json & arguments)
{
  if (!arguments.contains("frame_id") || !arguments.at("frame_id").is_string() ||
    !arguments.contains("poses") || !arguments.at("poses").is_array() ||
    arguments.at("poses").empty() || arguments.at("frame_id").get<std::string>().empty())
  {
    return ToolResult::failure("INVALID_ARGUMENTS", "目标坐标系不能为空，poses 必须为非空位姿数组");
  }
  const auto frame = arguments.at("frame_id").get<std::string>();
  const auto & poses = arguments.at("poses");
  ThroughPosesAction::Goal goal;
  goal.behavior_tree = through_poses_behavior_tree_;
  goal.poses.reserve(poses.size());
  const auto stamp = node_.now();
  for (const auto & input : poses) {
    if (!input.is_object() || !input.contains("x") || !input.at("x").is_number() ||
      !input.contains("y") || !input.at("y").is_number() ||
      !input.contains("yaw") || !input.at("yaw").is_number())
    {
      return ToolResult::failure("INVALID_ARGUMENTS", "每个位姿必须包含数值 x、y、yaw");
    }
    const auto x = input.at("x").get<double>();
    const auto y = input.at("y").get<double>();
    const auto yaw = input.at("yaw").get<double>();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      return ToolResult::failure("INVALID_ARGUMENTS", "每个位姿的坐标和朝向必须为有限数值");
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.header.stamp = stamp;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw / 2);
    pose.pose.orientation.w = std::cos(yaw / 2);
    goal.poses.push_back(std::move(pose));
  }
  return sendGoal(goal);
}

void NavigateThroughPosesStartNavigation::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult NavigateThroughPosesStartNavigation::execute(const Json & arguments)
{
  return navigation_->start_through_poses_navigation(arguments);
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::NavigateThroughPosesStartNavigation, vln_agent::ToolPluginBase)
