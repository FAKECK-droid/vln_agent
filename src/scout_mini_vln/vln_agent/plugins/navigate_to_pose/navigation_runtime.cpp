#include "navigation_runtime.hpp"

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

ToolResult NavigationRuntime::snapshot(const std::string & id) const
{
  const auto & task = tasks_.at(id);
  Json payload = task.feedback;
  payload["ok"] = true;
  payload["task_id"] = id;
  payload["status"] = task.status;
  return {payload, {}};
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
  std::string id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");
    }
    if (!active_.empty()) {
      return ToolResult::failure("NAVIGATION_BUSY", "已有活动导航任务");
    }
    id = "nav_" + std::to_string(++next_id_);
    tasks_.emplace(id, Task{});
    active_ = id;
    ++pending_goals_;
  }

  const auto weak = weak_from_this();
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [weak, id](GoalHandle::SharedPtr handle) {
      if (const auto self = weak.lock()) {
        {
          std::lock_guard<std::mutex> lock(self->mutex_);
          auto & task = self->tasks_.at(id);
          task.goal = handle;
          task.responded = true;
          --self->pending_goals_;
          if (!handle) {
            task.status = "rejected";
            if (self->active_ == id) {
              self->active_.clear();
            }
          }
        }
        self->condition_.notify_all();
      }
    };
  options.feedback_callback =
    [weak, id](GoalHandle::SharedPtr, const std::shared_ptr<const Action::Feedback> feedback) {
      if (const auto self = weak.lock()) {
        std::lock_guard<std::mutex> lock(self->mutex_);
        self->tasks_.at(id).feedback = {
          {"distance_remaining_m", feedback->distance_remaining},
          {"navigation_time_sec", rclcpp::Duration(feedback->navigation_time).seconds()},
          {"estimated_time_remaining_sec",
            rclcpp::Duration(feedback->estimated_time_remaining).seconds()},
          {"number_of_recoveries", feedback->number_of_recoveries}};
      }
    };
  options.result_callback = [weak, id](const GoalHandle::WrappedResult & result) {
      if (const auto self = weak.lock()) {
        {
          std::lock_guard<std::mutex> lock(self->mutex_);
          auto & task = self->tasks_.at(id);
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED: task.status = "succeeded"; break;
            case rclcpp_action::ResultCode::CANCELED: task.status = "canceled"; break;
            default: task.status = "failed"; break;
          }
          task.goal.reset();
          if (self->active_ == id) {
            self->active_.clear();
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
    tasks_.at(id).responded = true;
    tasks_.at(id).status = "failed";
    active_.clear();
    condition_.notify_all();
    throw;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [&] {return closed_ || tasks_.at(id).responded;});
  if (closed_) {
    return ToolResult::failure("TOOL_MANAGER_CLOSED", "等待导航目标回复时资源关闭");
  }
  if (tasks_.at(id).status == "rejected") {
    auto result = ToolResult::failure("NAVIGATION_REJECTED", "导航目标被服务端拒绝");
    result.payload["task_id"] = id;
    result.payload["status"] = "rejected";
    return result;
  }
  return snapshot(id);
}

ToolResult NavigationRuntime::get_feedback(const Json & arguments)
{
  const auto id = arguments.at("task_id").get<std::string>();
  std::lock_guard<std::mutex> lock(mutex_);
  if (tasks_.count(id) == 0) {
    return ToolResult::failure("TASK_NOT_FOUND", "未知导航任务: " + id);
  }
  return snapshot(id);
}

ToolResult NavigationRuntime::cancel_navigation(const Json & arguments)
{
  const auto id = arguments.at("task_id").get<std::string>();
  GoalHandle::SharedPtr goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");
    }
    if (tasks_.count(id) == 0) {
      return ToolResult::failure("TASK_NOT_FOUND", "未知导航任务: " + id);
    }
    const auto & task = tasks_.at(id);
    if (terminal(task.status) || task.status == "canceling") {
      return snapshot(id);
    }
    goal = task.goal;
    if (!goal) {
      return ToolResult::failure("NAVIGATION_NOT_READY", "目标尚未被接受");
    }
  }

  using CancelResponse = action_msgs::srv::CancelGoal::Response;
  struct Reply
  {
    CancelResponse::SharedPtr response;      ///< 本次取消调用的独立回复。
  };
  auto reply = std::make_shared<Reply>();
  const auto weak = weak_from_this();
  try {
    client_->async_cancel_goal(goal, [weak, id, reply](CancelResponse::SharedPtr response) {
        if (const auto self = weak.lock()) {
          {
            std::lock_guard<std::mutex> lock(self->mutex_);
            reply->response = response;
            auto & task = self->tasks_.at(id);
            if (response->return_code == CancelResponse::ERROR_NONE &&
              !response->goals_canceling.empty() && !terminal(task.status))
            {
              task.status = "canceling";
            }
          }
          self->condition_.notify_all();
        }
      });
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (terminal(tasks_.at(id).status)) {
      return snapshot(id);
    }
    throw;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [&] {return closed_ || reply->response != nullptr;});
  if (closed_) {
    return ToolResult::failure("TOOL_MANAGER_CLOSED", "等待取消回复时资源关闭");
  }
  if (terminal(tasks_.at(id).status)) {
    return snapshot(id);
  }
  if (reply->response->return_code != CancelResponse::ERROR_NONE ||
    reply->response->goals_canceling.empty())
  {
    return ToolResult::failure("NAVIGATION_CANCEL_REJECTED", "导航取消请求被拒绝");
  }
  return snapshot(id);
}

void NavigationRuntime::shutdown() noexcept
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (closed_) {
      return;
    }
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
    if (!active_.empty()) {
      goal = tasks_.at(active_).goal;
    }
  }
  if (goal) {
    try {
      client_->async_cancel_goal(goal);
    } catch (const std::exception & error) {
      RCLCPP_WARN(node_.get_logger(), "关闭导航资源时取消目标失败: %s", error.what());
    }
  }
}

}  // namespace vln_agent
