#include "vln_agent/plugins/tools/navigation/navigate_to_objects_surround.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{
ToolResult NavigationRuntime::navigate_to_objects_surround(RgbdRuntime & rgbd, const Json & arguments)
{
  return navigateToObjects(rgbd, arguments, true);
}

void NavigateToObjectsSurround::initialize(const ToolContext & context, const Json & settings)
{
  resources_ = context.resources;
  rgbd_resource_ = settings.value("rgbd_resource", std::string("rgbd/rgbd"));
  navigation_ = resources_->getOrCreate<NavigationRuntime>(context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult NavigateToObjectsSurround::execute(const Json & arguments)
{
  return navigation_->navigate_to_objects_surround(*resources_->get<RgbdRuntime>(rgbd_resource_), arguments);
}
}  // namespace vln_agent
PLUGINLIB_EXPORT_CLASS(vln_agent::NavigateToObjectsSurround, vln_agent::ToolPluginBase)
