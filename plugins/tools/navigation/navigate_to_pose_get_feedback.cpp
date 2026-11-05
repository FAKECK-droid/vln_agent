#include "vln_agent/plugins/tools/navigation/navigate_to_pose_get_feedback.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

namespace vln_agent
{

ToolResult NavigationRuntime::get_feedback(const Json & arguments)
{
  const auto id = arguments.at("task_id").get<std::string>();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!task_ || task_->id != id) {
    return ToolResult::failure("TASK_NOT_FOUND", "未知或已消费的导航任务: " + id);
  }
  // 使用独立引用，避免消费终态时重置 task_ 使参数失效。
  const auto task = task_;
  return snapshot(task);
}


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
