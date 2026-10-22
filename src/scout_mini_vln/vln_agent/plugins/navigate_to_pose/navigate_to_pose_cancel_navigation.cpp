#include "navigate_to_pose_cancel_navigation.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "navigation_runtime.hpp"

namespace vln_agent
{

void NavigateToPoseCancelNavigation::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult NavigateToPoseCancelNavigation::execute(const Json & arguments)
{
  return navigation_->cancel_navigation(arguments);
}

void NavigateToPoseCancelNavigation::shutdown() noexcept
{
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::NavigateToPoseCancelNavigation, vln_agent::ToolPluginBase)
