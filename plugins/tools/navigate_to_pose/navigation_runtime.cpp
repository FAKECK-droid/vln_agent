#include "navigation_runtime.hpp"
#include "vln_agent/utils/time.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include <action_msgs/srv/cancel_goal.hpp>

#include "vln_agent/tools/ros_callbacks.hpp"

namespace vln_agent
{

NavigationRuntime::NavigationRuntime(rclcpp::Node & node, const Json & settings)
: node_(node),
  callbacks_(std::make_unique<ToolCallbacks>(node)),
  client_(rclcpp_action::create_client<Action>(
      &node, settings.value("action_name", std::string("/navigate_to_pose")), callbacks_->group())),
  shutdown_timeout_sec_(settings.value("shutdown_timeout_sec", 2.0))
{
  if (shutdown_timeout_sec_ < 0) {
    throw std::invalid_argument("shutdown_timeout_sec 不能小于零");
  }
}

NavigationRuntime::~NavigationRuntime()
{
  shutdown();
}

bool NavigationRuntime::terminal(const std::string & status)
{
  return status == "succeeded" || status == "failed" ||
         status == "canceled" || status == "rejected";
}

ToolResult NavigationRuntime::snapshot(const std::shared_ptr<Task> & task)
{
  Json payload = task->feedback;
  payload["ok"] = true;
  payload["time_stamp"] = utils::utcNow();
  payload["task_id"] = task->id;
  payload["status"] = task->status;
  if (terminal(task->status) && task_ == task) {task_.reset();}
  return {std::move(payload), {}};
}

ToolResult NavigationRuntime::start_navigation(const Json & arguments)
{
  Action::Goal goal;
  goal.pose.header.frame_id = arguments.at("frame_id").get<std::string>();
  goal.pose.header.stamp = node_.now();
  goal.pose.pose.position.x = arguments.at("x").get<double>();
  goal.pose.pose.position.y = arguments.at("y").get<double>();
  const auto yaw = arguments.at("yaw").get<double>();
  goal.pose.pose.orientation.z = std::sin(yaw / 2);
  goal.pose.pose.orientation.w = std::cos(yaw / 2);
  if (goal.pose.header.frame_id.empty() || !std::isfinite(goal.pose.pose.position.x) ||
    !std::isfinite(goal.pose.pose.position.y) || !std::isfinite(yaw))
  {
    return ToolResult::failure("INVALID_ARGUMENTS", "目标坐标系不能为空，坐标和朝向必须为有限数值");
  }
  if (!client_->action_server_is_ready()) {
    return ToolResult::failure("NAVIGATION_UNAVAILABLE", "导航 Action 服务尚未就绪");
  }

  std::shared_ptr<Task> task;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");}
    if (task_) {
      if (terminal(task_->status)) {
        auto result = ToolResult::failure(
          "NAVIGATION_RESULT_PENDING", "请先调用 get_feedback 读取上次导航终态");
        result.payload["task_id"] = task_->id;
        return result;
      }
      return ToolResult::failure("NAVIGATION_BUSY", "已有活动导航任务");
    }
    task = std::make_shared<Task>();
    task->id = "nav_" + std::to_string(++next_id_);
    task_ = task;
    ++pending_goals_;
  }

  const auto weak = weak_from_this();
  const std::weak_ptr<Task> weak_task = task;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [weak, weak_task](GoalHandle::SharedPtr handle) {
      if (const auto self = weak.lock()) {
        {
          std::lock_guard<std::mutex> lock(self->mutex_);
          --self->pending_goals_;
          if (const auto task = weak_task.lock()) {
            task->goal = handle;
            task->responded = true;
            if (!handle) {task->status = "rejected";}
          }
        }
        self->condition_.notify_all();
      }
    };
  options.feedback_callback =
    [weak, weak_task](GoalHandle::SharedPtr, const std::shared_ptr<const Action::Feedback> feedback) {
      if (const auto self = weak.lock()) {
        std::lock_guard<std::mutex> lock(self->mutex_);
        if (const auto task = weak_task.lock(); task && !terminal(task->status)) {
          task->feedback = {
            {"distance_remaining_m", feedback->distance_remaining},
            {"navigation_time_sec", rclcpp::Duration(feedback->navigation_time).seconds()},
            {"estimated_time_remaining_sec",
              rclcpp::Duration(feedback->estimated_time_remaining).seconds()},
            {"number_of_recoveries", feedback->number_of_recoveries}};
        }
      }
    };
  options.result_callback = [weak, weak_task](const GoalHandle::WrappedResult & result) {
      if (const auto self = weak.lock()) {
        {
          std::lock_guard<std::mutex> lock(self->mutex_);
          if (const auto task = weak_task.lock()) {
            switch (result.code) {
              case rclcpp_action::ResultCode::SUCCEEDED: task->status = "succeeded"; break;
              case rclcpp_action::ResultCode::CANCELED: task->status = "canceled"; break;
              default: task->status = "failed"; break;
            }
            task->goal.reset();
          }
        }
        self->condition_.notify_all();
      }
    };
  try {
    client_->async_send_goal(goal, options);
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    --pending_goals_;
    task->responded = true;
    if (task_ == task) {task_.reset();}
    condition_.notify_all();
    throw;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [&] {return closed_ || task->responded;});
  if (closed_) {
    return ToolResult::failure("TOOL_MANAGER_CLOSED", "等待导航目标回复时资源关闭");
  }
  if (task->status == "rejected") {
    auto result = ToolResult::failure("NAVIGATION_REJECTED", "导航目标被服务端拒绝");
    result.payload["task_id"] = task->id;
    result.payload["status"] = task->status;
    if (task_ == task) {task_.reset();}
    return result;
  }
  return snapshot(task);
}

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

ToolResult NavigationRuntime::cancel_navigation(const Json & arguments)
{
  const auto id = arguments.at("task_id").get<std::string>();
  std::shared_ptr<Task> task;
  GoalHandle::SharedPtr goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");}
    if (!task_ || task_->id != id) {
      return ToolResult::failure("TASK_NOT_FOUND", "未知或已消费的导航任务: " + id);
    }
    task = task_;
    if (terminal(task->status) || task->status == "canceling") {return snapshot(task);}
    goal = task->goal;
    if (!goal) {return ToolResult::failure("NAVIGATION_NOT_READY", "目标尚未被接受");}
  }

  using CancelResponse = action_msgs::srv::CancelGoal::Response;
  struct Reply {CancelResponse::SharedPtr response;};
  auto reply = std::make_shared<Reply>();
  const auto weak = weak_from_this();
  const std::weak_ptr<Task> weak_task = task;
  try {
    client_->async_cancel_goal(goal, [weak, weak_task, reply](CancelResponse::SharedPtr response) {
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
      });
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

void NavigationRuntime::shutdown() noexcept
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (closed_) {return;}
    closed_ = true;
    condition_.notify_all();
    if (!condition_.wait_for(lock, std::chrono::duration<double>(shutdown_timeout_sec_),
        [this] {return pending_goals_ == 0;}))
    {
      RCLCPP_WARN(node_.get_logger(), "关闭导航资源时仍有未确认目标，无法确认其物理状态");
    }
  }
  callbacks_->shutdown();
  GoalHandle::SharedPtr goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (task_ && !terminal(task_->status)) {goal = task_->goal;}
    task_.reset();
  }
  if (goal) {
    try {client_->async_cancel_goal(goal);}
    catch (const std::exception & error) {
      RCLCPP_WARN(node_.get_logger(), "关闭导航资源时取消目标失败: %s", error.what());
    }
  }
}

}  // namespace vln_agent
