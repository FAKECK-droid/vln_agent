#include "vln_agent/plugins/tools/navigation/navigate_through_pixels_start_navigation.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{
ToolResult NavigationRuntime::navigate_through_pixels_start_navigation(const Json & arguments)
{
  return startPixelNavigation(arguments, true);
}

void NavigateThroughPixelsStartNavigation::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult NavigateThroughPixelsStartNavigation::execute(const Json & arguments)
{
  return navigation_->navigate_through_pixels_start_navigation(arguments);
}
}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::NavigateThroughPixelsStartNavigation, vln_agent::ToolPluginBase)
