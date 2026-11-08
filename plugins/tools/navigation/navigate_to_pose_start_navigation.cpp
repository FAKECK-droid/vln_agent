#include "vln_agent/plugins/tools/navigation/navigate_to_pose_start_navigation.hpp"

#include <chrono>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"
#include "vln_agent/utils/time.hpp"

namespace vln_agent
{

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
  return sendGoal(goal, false);
}

ToolResult NavigationRuntime::sendGoal(const Action::Goal & goal, bool release_on_finish)
{
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
        result.payload["data"]["task_id"] = task_->id;
        return result;
      }
      return ToolResult::failure("NAVIGATION_BUSY", "已有活动导航任务");
    }
    task = std::make_shared<Task>();
    task->id = "nav_" + std::to_string(++next_id_);
    task->release_on_finish = release_on_finish;
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
  if (!release_on_finish) {
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
            const auto & current = feedback->current_pose;
            if (!current.header.frame_id.empty()) {
              auto pose = utils::posePayload(current.header.frame_id,
                current.pose.position.x, current.pose.position.y, tf2::getYaw(current.pose.orientation));
              bool simulated = false;
              self->node_.get_parameter_or("use_sim_time", simulated, false);
              if (!simulated && (current.header.stamp.sec != 0 || current.header.stamp.nanosec != 0)) {
                pose["time_stamp"] = utils::formatUtcTime(std::chrono::system_clock::time_point(
                  std::chrono::seconds(current.header.stamp.sec) +
                  std::chrono::nanoseconds(current.header.stamp.nanosec)));
              }
              task->pose = std::move(pose);
            }
          }
        }
      };
  }
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
            if (task->release_on_finish && self->task_ == task) {self->task_.reset();}
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
    result.payload["data"]["task_id"] = task->id;
    result.payload["data"]["status"] = task->status;
    if (task_ == task) {task_.reset();}
    return result;
  }
  return snapshot(task);
}


void NavigateToPoseStartNavigation::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult NavigateToPoseStartNavigation::execute(const Json & arguments)
{
  return navigation_->start_navigation(arguments);
}

void NavigateToPoseStartNavigation::shutdown() noexcept
{
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::NavigateToPoseStartNavigation, vln_agent::ToolPluginBase)
