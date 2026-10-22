#include "navigate_to_pose_get_feedback.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "navigation_runtime.hpp"

namespace vln_agent
{

void NavigateToPoseGetFeedback::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult NavigateToPoseGetFeedback::execute(const Json & arguments)
{
  return navigation_->get_feedback(arguments);
}

void NavigateToPoseGetFeedback::shutdown() noexcept
{
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::NavigateToPoseGetFeedback, vln_agent::ToolPluginBase)
