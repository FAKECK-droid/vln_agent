#include "vln_agent/plugins/tools/navigation/evaluate_navigation_goals.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{
void EvaluateNavigationGoals::initialize(const ToolContext & context, const Json & settings)
{
  resources_ = context.resources;
  rgbd_resource_ = context.group_settings.value("rgbd_resource", std::string("rgbd/rgbd"));
  scoring_ = navigation::GoalScoring::fromSettings(settings);
  navigation_ = resources_->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult EvaluateNavigationGoals::execute(const Json & arguments)
{
  const auto rgbd = resources_->get<RgbdRuntime>(rgbd_resource_);
  return navigation_->evaluateImageGoals(*rgbd, arguments, scoring_);
}
}  // namespace vln_agent
PLUGINLIB_EXPORT_CLASS(vln_agent::EvaluateNavigationGoals, vln_agent::ToolPluginBase)
