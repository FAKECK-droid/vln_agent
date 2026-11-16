#include "vln_agent/plugins/tools/navigation/get_navigation_feedback.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

namespace vln_agent
{

ToolResult NavigationRuntime::get_feedback(const Json & arguments)
{
  const auto id = arguments.at("task_id").get<std::string>();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!task_ || task_->id != id) {
    return ToolResult::failure("TASK_NOT_FOUND", "未知或已结束的导航任务: " + id);
  }
  return snapshot(task_);
}

void GetNavigationFeedback::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult GetNavigationFeedback::execute(const Json & arguments)
{
  return navigation_->get_feedback(arguments);
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::GetNavigationFeedback, vln_agent::ToolPluginBase)
