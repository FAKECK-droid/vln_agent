#include "vln_agent/plugins/tools/navigation/get_surroundings.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <opencv2/imgcodecs.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include "vln_agent/utils/base64.hpp"
#include "vln_agent/utils/time.hpp"

namespace vln_agent
{
ToolResult NavigationRuntime::get_surroundings(const Json & arguments)
{
  const auto source = arguments.at("costmap").get<std::string>();
  const auto client = source == "local" ? local_costmap_client_ : global_costmap_client_;
  const auto timeout = std::chrono::duration<double>(costmap_timeout_sec_);
  if (!client->wait_for_service(timeout)) {
    return ToolResult::failure("COSTMAP_UNAVAILABLE", "代价地图服务不可用");
  }
  auto future = client->async_send_request(std::make_shared<nav2_msgs::srv::GetCostmap::Request>());
  if (future.wait_for(timeout) != std::future_status::ready) {
    client->remove_pending_request(future);
    return ToolResult::failure("COSTMAP_TIMEOUT", "等待代价地图服务响应超时");
  }
  const auto response = future.get();
  const auto & costmap = response->map;
  if (costmap.metadata.size_x == 0 || costmap.metadata.size_y == 0) {
    return ToolResult::failure("EMPTY_COSTMAP", "代价地图尚无数据");
  }
  const auto map_from_source = tf_buffer_->lookupTransform(
    map_frame_, costmap.header.frame_id, tf2::TimePointZero);
  const auto robot = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  const double yaw = tf2::getYaw(map_from_source.transform.rotation);
  const auto & origin = costmap.metadata.origin;
  navigation::Pose2D origin_in_map{
    map_from_source.transform.translation.x + std::cos(yaw) * origin.position.x -
    std::sin(yaw) * origin.position.y,
    map_from_source.transform.translation.y + std::sin(yaw) * origin.position.x +
    std::cos(yaw) * origin.position.y,
    yaw + tf2::getYaw(origin.orientation)};
  navigation::Environment environment;
  environment.frame_id = map_frame_;
  environment.robot = {robot.transform.translation.x, robot.transform.translation.y,
    tf2::getYaw(robot.transform.rotation)};
  environment.range_m = arguments.at("range_m").get<double>();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    environment.id = "env_" + std::to_string(++next_environment_id_);
  }
  const auto image = navigation::renderSurroundings(
    costmap, origin_in_map, environment, image_size_, source);
  std::vector<unsigned char> png;
  cv::imencode(".png", image, png, {cv::IMWRITE_PNG_COMPRESSION, 1});
  const auto image_url = "data:image/png;base64," + utils::encodeBase64(png);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");}
    environments_.push_back(environment);
    while (environments_.size() > max_environments_) {environments_.pop_front();}
  }
  const auto & rect = environment.map_rect;
  auto result = ToolResult::success({{"environment_id", environment.id}});
  result.payload["pose"] = utils::posePayload(map_frame_,
    environment.robot.x, environment.robot.y, environment.robot.yaw);
  bool simulated = false;
  node_.get_parameter_or("use_sim_time", simulated, false);
  const auto stamp = robot.header.stamp;
  if (!simulated && (stamp.sec != 0 || stamp.nanosec != 0)) {
    result.payload["pose"]["time_stamp"] = utils::formatUtcTime(
      std::chrono::system_clock::time_point(std::chrono::seconds(stamp.sec) +
      std::chrono::nanoseconds(stamp.nanosec)));
  }
  Json description{{"metadata", {
    {"costmap", source}, {"range_m", environment.range_m},
    {"image_width", image.cols}, {"image_height", image.rows},
    {"map_rect", {{"x", rect.x}, {"y", rect.y}, {"width", rect.width}, {"height", rect.height}}},
    {"meters_per_pixel", environment.meters_per_pixel},
    {"pixel_origin", "top_left"}, {"image_up", "robot_forward"},
    {"image_left", "robot_left"}, {"yaw_convention", "0=up, positive=counterclockwise"}}}};
  if (!simulated && (costmap.header.stamp.sec != 0 || costmap.header.stamp.nanosec != 0)) {
    description["time_stamp"] = utils::formatUtcTime(std::chrono::system_clock::time_point(
      std::chrono::seconds(costmap.header.stamp.sec) +
      std::chrono::nanoseconds(costmap.header.stamp.nanosec)));
  }
  result.payload["images"] = Json::array({std::move(description)});
  result.images.push_back({{"type", "input_image"}, {"image_url", image_url}, {"detail", "high"}});
  return result;
}

void GetSurroundings::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult GetSurroundings::execute(const Json & arguments)
{
  return navigation_->get_surroundings(arguments);
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::GetSurroundings, vln_agent::ToolPluginBase)
