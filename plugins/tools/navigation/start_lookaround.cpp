#include "vln_agent/plugins/tools/navigation/start_lookaround.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/buffer.h>

namespace vln_agent
{
ToolResult NavigationRuntime::start_lookaround()
{
  const auto robot = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  Action::Goal goal;
  goal.pose.header.frame_id = map_frame_;
  goal.pose.header.stamp = node_.now();
  goal.pose.pose.position.x = robot.transform.translation.x;
  goal.pose.pose.position.y = robot.transform.translation.y;
  goal.pose.pose.position.z = robot.transform.translation.z;
  goal.pose.pose.orientation = robot.transform.rotation;
  goal.behavior_tree = lookaround_behavior_tree_.empty() ?
    ament_index_cpp::get_package_share_directory("vln_behavior_tree") +
    "/behavior_trees/lookaround.xml" : lookaround_behavior_tree_;
  return sendGoal(goal);
}

void StartLookaround::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult StartLookaround::execute(const Json &)
{
  return navigation_->start_lookaround();
}
}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::StartLookaround, vln_agent::ToolPluginBase)
