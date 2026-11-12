#include "vln_agent/plugins/tools/navigation/navigate_through_poses_get_feedback.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{

ToolResult NavigationRuntime::get_through_poses_feedback(const Json & arguments)
{
  return getFeedback(arguments, true);
}

void NavigateThroughPosesGetFeedback::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult NavigateThroughPosesGetFeedback::execute(const Json & arguments)
{
  return navigation_->get_through_poses_feedback(arguments);
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::NavigateThroughPosesGetFeedback, vln_agent::ToolPluginBase)
