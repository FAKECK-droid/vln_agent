#include "vln_agent/plugins/tools/navigation/cancel_lookaround.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{
void CancelLookaround::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult CancelLookaround::execute(const Json & arguments)
{
  return navigation_->cancel_navigation(arguments);
}
}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::CancelLookaround, vln_agent::ToolPluginBase)
