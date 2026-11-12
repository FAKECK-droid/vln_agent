#include "vln_agent/plugins/tools/navigation/navigate_to_pose_cancel_navigation.hpp"

#include <action_msgs/srv/cancel_goal.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

namespace vln_agent
{

ToolResult NavigationRuntime::cancel_navigation(const Json & arguments)
{
  return cancelNavigation(arguments, false);
}

ToolResult NavigationRuntime::cancelNavigation(const Json & arguments, bool through_poses)
{
  const auto id = arguments.at("task_id").get<std::string>();
  std::shared_ptr<Task> task;
  GoalHandle::SharedPtr goal;
  ThroughPosesGoalHandle::SharedPtr through_poses_goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");}
    if (!task_ || task_->id != id || task_->through_poses != through_poses) {
      return ToolResult::failure("TASK_NOT_FOUND", "未知或已消费的导航任务: " + id);
    }
    task = task_;
    if (terminal(task->status) || task->status == "canceling") {return snapshot(task);}
    goal = task->goal;
    through_poses_goal = task->through_poses_goal;
    if (!goal && !through_poses_goal) {return ToolResult::failure("NAVIGATION_NOT_READY", "目标尚未被接受");}
  }

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
    if (through_poses) {through_poses_client_->async_cancel_goal(through_poses_goal, callback);}
    else {client_->async_cancel_goal(goal, callback);}
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (terminal(task->status)) {return snapshot(task);}
    throw;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [&] {return closed_ || reply->response != nullptr;});
  if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "等待取消回复时资源关闭");}
  if (terminal(task->status)) {return snapshot(task);}
  if (reply->response->return_code != CancelResponse::ERROR_NONE ||
    reply->response->goals_canceling.empty())
  {
    return ToolResult::failure("NAVIGATION_CANCEL_REJECTED", "导航取消请求被拒绝");
  }
  return snapshot(task);
}


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
