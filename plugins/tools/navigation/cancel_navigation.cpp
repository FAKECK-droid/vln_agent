#include "vln_agent/plugins/tools/navigation/cancel_navigation.hpp"

#include <action_msgs/srv/cancel_goal.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

namespace vln_agent
{

ToolResult NavigationRuntime::cancel_navigation(const Json & arguments)
{
  const auto id = arguments.at("task_id").get<std::string>();
  std::unique_lock<std::mutex> lock(mutex_);
  if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");}
  if (!task_ || task_->id != id) {
    return ToolResult::failure("TASK_NOT_FOUND", "未知或已消费的导航任务: " + id);
  }
  const auto task = task_;
  const auto wait_for_terminal = [&] {
      condition_.wait(lock, [&] {return closed_ || terminal(task->status);});
      if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "等待取消终态时资源关闭");}
      return snapshot(task);
    };
  if (terminal(task->status) || task->status == "canceling") {return wait_for_terminal();}
  const auto cancel = task->cancel;
  if (!cancel) {return ToolResult::failure("NAVIGATION_NOT_READY", "目标尚未被接受");}
  lock.unlock();

  using CancelResponse = action_msgs::srv::CancelGoal::Response;
  struct Reply {CancelResponse::SharedPtr response;};
  auto reply = std::make_shared<Reply>();
  const auto weak = weak_from_this();
  const std::weak_ptr<Task> weak_task = task;
  try {
    const auto callback = [weak, weak_task, reply](CancelResponse::SharedPtr response) {
        if (const auto self = weak.lock()) {
          {
            std::lock_guard<std::mutex> lock(self->mutex_);
            reply->response = response;
            if (const auto task = weak_task.lock(); task &&
              response->return_code == CancelResponse::ERROR_NONE &&
              !response->goals_canceling.empty() && !terminal(task->status))
            {
              task->status = "canceling";
            }
          }
          self->condition_.notify_all();
        }
      };
    cancel(callback);
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
    lock.lock();
    if (closed_ || terminal(task->status)) {return wait_for_terminal();}
    throw;
  }

  lock.lock();
  condition_.wait(lock, [&] {return closed_ || terminal(task->status) || reply->response != nullptr;});
  if (!closed_ && !terminal(task->status) && task->status != "canceling" &&
    reply->response->return_code != CancelResponse::ERROR_GOAL_TERMINATED)
  {
    return ToolResult::failure("NAVIGATION_CANCEL_REJECTED", "导航取消请求被拒绝");
  }
  return wait_for_terminal();
}

void CancelNavigation::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult CancelNavigation::execute(const Json & arguments)
{
  return navigation_->cancel_navigation(arguments);
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::CancelNavigation, vln_agent::ToolPluginBase)
