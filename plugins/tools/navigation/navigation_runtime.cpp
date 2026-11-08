#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"
#include "vln_agent/utils/time.hpp"

#include <chrono>
#include <stdexcept>


#include "vln_agent/tools/ros_callbacks.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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

  global_costmap_client_ = node_.create_client<nav2_msgs::srv::GetCostmap>(
    settings.value("global_costmap_service", std::string("/global_costmap/get_costmap")),
    rmw_qos_profile_services_default, callbacks_->group());
  local_costmap_client_ = node_.create_client<nav2_msgs::srv::GetCostmap>(
    settings.value("local_costmap_service", std::string("/local_costmap/get_costmap")),
    rmw_qos_profile_services_default, callbacks_->group());
  map_frame_ = settings.value("map_frame", std::string("map"));
  base_frame_ = settings.value("base_frame", std::string("base_link"));
  lookaround_behavior_tree_ = settings.value("lookaround_behavior_tree", std::string{});
  costmap_timeout_sec_ = settings.value("costmap_timeout_sec", 2.0);
  image_size_ = settings.value("image_size", 512);
  max_environments_ = settings.value("max_environments", std::size_t{8});
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
  rclcpp::SubscriptionOptions options;
  options.callback_group = callbacks_->group();
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
    *tf_buffer_, &node_, false, tf2_ros::DynamicListenerQoS(),
    tf2_ros::StaticListenerQoS(), options, options);
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
  if (terminal(task->status) && task_ == task) {task_.reset();}
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
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (task_ && !terminal(task_->status)) {goal = task_->goal;}
    task_.reset();
    environments_.clear();
  }
  if (goal) {
    try {client_->async_cancel_goal(goal);}
    catch (const std::exception & error) {
      RCLCPP_WARN(node_.get_logger(), "关闭导航资源时取消目标失败: %s", error.what());
    }
  }
}

}  // namespace vln_agent
