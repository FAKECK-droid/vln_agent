#include "vln_agent/plugins/tools/navigation/navigate_through_poses_cancel_navigation.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{

ToolResult NavigationRuntime::cancel_through_poses_navigation(const Json & arguments)
{
  return cancelNavigation(arguments, true);
}

void NavigateThroughPosesCancelNavigation::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult NavigateThroughPosesCancelNavigation::execute(const Json & arguments)
{
  return navigation_->cancel_through_poses_navigation(arguments);
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::NavigateThroughPosesCancelNavigation, vln_agent::ToolPluginBase)
