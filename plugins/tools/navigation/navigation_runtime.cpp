#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"
#include "vln_agent/utils/time.hpp"

#include <chrono>
#include <stdexcept>
#include <type_traits>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "vln_agent/tools/ros_callbacks.hpp"

namespace vln_agent
{

NavigationRuntime::NavigationRuntime(rclcpp::Node & node, const Json & settings)
: node_(node),
  callbacks_(std::make_unique<ToolCallbacks>(node)),
  client_(rclcpp_action::create_client<Action>(
      &node, settings.value("action_name", std::string("/navigate_to_pose")), callbacks_->group())),
  through_poses_client_(rclcpp_action::create_client<ThroughPosesAction>(
      &node, settings.value("navigate_through_poses_action_name",
      std::string("/navigate_through_poses")), callbacks_->group())),
  shutdown_timeout_sec_(settings.value("shutdown_timeout_sec", 2.0))
{
  if (shutdown_timeout_sec_ < 0) {
    throw std::invalid_argument("shutdown_timeout_sec 不能小于零");
  }

  global_costmap_client_ = node_.create_client<nav2_msgs::srv::GetCostmap>(
    settings.value("global_costmap_service", std::string("/global_costmap/get_costmap")),
    rmw_qos_profile_services_default, callbacks_->group());
  local_costmap_client_ = node_.create_client<nav2_msgs::srv::GetCostmap>(
    settings.value("local_costmap_service", std::string("/local_costmap/get_costmap")),
    rmw_qos_profile_services_default, callbacks_->group());
  map_frame_ = settings.value("map_frame", std::string("map"));
  base_frame_ = settings.value("base_frame", std::string("base_link"));
  lookaround_behavior_tree_ = settings.value("lookaround_behavior_tree", std::string{});
  through_poses_behavior_tree_ = settings.value("navigate_through_poses_behavior_tree",
    std::string("/home/yan/ros2_ws/src/scout_mini_vln/vln_behavior_tree/behavior_trees/vln_navthroughgoals.xml"));
  if (through_poses_behavior_tree_.empty()) {
    throw std::invalid_argument("navigate_through_poses_behavior_tree 不能为空");
  }
  costmap_timeout_sec_ = settings.value("costmap_timeout_sec", 2.0);
  image_size_ = settings.value("image_size", 512);
  max_environments_ = settings.value("max_environments", std::size_t{8});
  const auto max_pixel_goals = settings.value("max_pixel_goals", Json(8));
  if (!max_pixel_goals.is_number_integer() || max_pixel_goals <= 0) {
    throw std::invalid_argument("max_pixel_goals 必须为正整数");
  }
  max_pixel_goals_ = max_pixel_goals.get<std::size_t>();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
  rclcpp::SubscriptionOptions options;
  options.callback_group = callbacks_->group();
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
    *tf_buffer_, &node_, false, tf2_ros::DynamicListenerQoS(),
    tf2_ros::StaticListenerQoS(), options, options);
}

template<typename NavigationAction>
ToolResult NavigationRuntime::sendActionGoal(
  const typename NavigationAction::Goal & goal,
  const typename rclcpp_action::Client<NavigationAction>::SharedPtr & client)
{
  using Handle = rclcpp_action::ClientGoalHandle<NavigationAction>;
  constexpr bool through_poses = std::is_same_v<NavigationAction, ThroughPosesAction>;
  if (!client->action_server_is_ready()) {
    return ToolResult::failure("NAVIGATION_UNAVAILABLE", "导航 Action 服务尚未就绪");
  }

  std::shared_ptr<Task> task;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");}
    if (task_) {
      return ToolResult::failure("NAVIGATION_BUSY", "已有活动导航任务");
    }
    task = std::make_shared<Task>();
    task->through_poses = through_poses;
    task->id = std::string(through_poses ? "nav_through_poses_" : "nav_") +
      std::to_string(++next_id_);
    task_ = task;
    ++pending_goals_;
  }

  const auto weak = weak_from_this();
  const std::weak_ptr<Task> weak_task = task;
  typename rclcpp_action::Client<NavigationAction>::SendGoalOptions options;
  options.goal_response_callback = [weak, weak_task](typename Handle::SharedPtr handle) {
      if (const auto self = weak.lock()) {
        {
          std::lock_guard<std::mutex> lock(self->mutex_);
          --self->pending_goals_;
          if (const auto task = weak_task.lock()) {
            if constexpr (through_poses) {task->through_poses_goal = handle;}
            else {task->goal = handle;}
            task->responded = true;
            if (!handle) {task->status = "rejected";}
          }
        }
        self->condition_.notify_all();
      }
    };
  options.feedback_callback =
    [weak, weak_task](typename Handle::SharedPtr,
      const std::shared_ptr<const typename NavigationAction::Feedback> feedback) {
      if (const auto self = weak.lock()) {
        std::lock_guard<std::mutex> lock(self->mutex_);
        if (const auto task = weak_task.lock(); task && !terminal(task->status)) {
          task->feedback = {
            {"distance_remaining_m", feedback->distance_remaining},
            {"navigation_time_sec", rclcpp::Duration(feedback->navigation_time).seconds()},
            {"estimated_time_remaining_sec",
              rclcpp::Duration(feedback->estimated_time_remaining).seconds()},
            {"number_of_recoveries", feedback->number_of_recoveries}};
          if constexpr (through_poses) {
            task->feedback["number_of_poses_remaining"] = feedback->number_of_poses_remaining;
          }
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
  options.result_callback = [weak, weak_task](const typename Handle::WrappedResult & result) {
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
            task->through_poses_goal.reset();
            if (self->task_ == task) {self->task_.reset();}
          }
        }
        self->condition_.notify_all();
      }
    };
  try {
    client->async_send_goal(goal, options);
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

ToolResult NavigationRuntime::sendGoal(const Action::Goal & goal)
{
  return sendActionGoal<Action>(goal, client_);
}

ToolResult NavigationRuntime::sendGoal(const ThroughPosesAction::Goal & goal)
{
  return sendActionGoal<ThroughPosesAction>(goal, through_poses_client_);
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
  payload["task_id"] = task->id;
  payload["status"] = task->status;
  auto result = ToolResult::success(std::move(payload));
  if (!task->pose.is_null()) {result.payload["pose"] = task->pose;}
  return result;
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
  global_costmap_client_->prune_pending_requests();
  local_costmap_client_->prune_pending_requests();
  tf_listener_.reset();
  GoalHandle::SharedPtr goal;
  ThroughPosesGoalHandle::SharedPtr through_poses_goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (task_ && !terminal(task_->status)) {
      goal = task_->goal;
      through_poses_goal = task_->through_poses_goal;
    }
    task_.reset();
    environments_.clear();
  }
  if (goal) {
    try {client_->async_cancel_goal(goal);}
    catch (const std::exception & error) {
      RCLCPP_WARN(node_.get_logger(), "关闭导航资源时取消目标失败: %s", error.what());
    }
  }
  if (through_poses_goal) {
    try {through_poses_client_->async_cancel_goal(through_poses_goal);}
    catch (const std::exception & error) {
      RCLCPP_WARN(node_.get_logger(), "关闭导航资源时取消多点目标失败: %s", error.what());
    }
  }
}

}  // namespace vln_agent
