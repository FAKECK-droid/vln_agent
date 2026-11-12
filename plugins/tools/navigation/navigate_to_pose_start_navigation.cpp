#include "vln_agent/plugins/tools/navigation/navigate_to_pose_start_navigation.hpp"

#include <cmath>
#include <pluginlib/class_list_macros.hpp>

#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

namespace vln_agent
{

ToolResult NavigationRuntime::start_navigation(const Json & arguments)
{
  Action::Goal goal;
  goal.pose.header.frame_id = arguments.at("frame_id").get<std::string>();
  goal.pose.header.stamp = node_.now();
  goal.pose.pose.position.x = arguments.at("x").get<double>();
  goal.pose.pose.position.y = arguments.at("y").get<double>();
  const auto yaw = arguments.at("yaw").get<double>();
  goal.pose.pose.orientation.z = std::sin(yaw / 2);
  goal.pose.pose.orientation.w = std::cos(yaw / 2);
  if (goal.pose.header.frame_id.empty() || !std::isfinite(goal.pose.pose.position.x) ||
    !std::isfinite(goal.pose.pose.position.y) || !std::isfinite(yaw))
  {
    return ToolResult::failure("INVALID_ARGUMENTS", "目标坐标系不能为空，坐标和朝向必须为有限数值");
  }
  return sendGoal(goal);
}

void NavigateToPoseStartNavigation::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult NavigateToPoseStartNavigation::execute(const Json & arguments)
{
  return navigation_->start_navigation(arguments);
}

void NavigateToPoseStartNavigation::shutdown() noexcept
{
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::NavigateToPoseStartNavigation, vln_agent::ToolPluginBase)
