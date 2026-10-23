#include "navigate_to_pose_start_navigation.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "navigation_runtime.hpp"

namespace vln_agent
{

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
