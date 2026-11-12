#include "vln_agent/plugins/tools/navigation/navigate_to_pixel_start_navigation.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{
ToolResult NavigationRuntime::navigate_to_pixel_start_navigation(const Json & arguments)
{
  return startPixelNavigation(arguments, false);
}

ToolResult NavigationRuntime::startPixelNavigation(const Json & arguments, bool through_pixels)
{
  if (!arguments.contains("environment_id") || !arguments.at("environment_id").is_string() ||
    arguments.at("environment_id").get<std::string>().empty())
  {
    return ToolResult::failure("INVALID_ARGUMENTS", "environment_id 必须为非空字符串");
  }
  const auto id = arguments.at("environment_id").get<std::string>();
  if (through_pixels && (!arguments.contains("pixels") || !arguments.at("pixels").is_array() ||
    arguments.at("pixels").empty() || arguments.at("pixels").size() > max_pixel_goals_))
  {
    auto result = ToolResult::failure("INVALID_ARGUMENTS",
      "pixels 必须为非空数组，像素点数量最多为 " + std::to_string(max_pixel_goals_));
    result.payload["data"] = {{"environment_id", id}, {"max_pixel_goals", max_pixel_goals_}};
    return result;
  }

  navigation::Environment environment;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = std::find_if(environments_.begin(), environments_.end(),
      [&](const auto & item) {return item.id == id;});
    if (found == environments_.end()) {
      return ToolResult::failure("ENVIRONMENT_NOT_FOUND", "环境 ID 已消费或被更新淘汰: " + id);
    }
    environment = *found;
  }
  // 多点批次只借用输入数组；数量检查先于批次转换与 Action 请求。
  const Json single = through_pixels ? Json() : Json::array({arguments});
  const auto & pixels = through_pixels ? arguments.at("pixels") : single;
  Json poses = Json::array();
  for (const auto & pixel : pixels) {
    if (!pixel.is_object() || !pixel.contains("pixel_x") || !pixel.at("pixel_x").is_number() ||
      !pixel.contains("pixel_y") || !pixel.at("pixel_y").is_number() ||
      !pixel.contains("yaw") || !pixel.at("yaw").is_number())
    {
      return ToolResult::failure("INVALID_ARGUMENTS", "每个像素点必须包含数值 pixel_x、pixel_y、yaw");
    }
    const auto image_yaw = pixel.at("yaw").get<double>();
    const auto yaw = environment.robot.yaw + image_yaw;
    if (!std::isfinite(image_yaw) || !std::isfinite(yaw)) {
      return ToolResult::failure("INVALID_ARGUMENTS", "像素目标朝向必须为有限数值");
    }
    try {
      const auto point = navigation::pixelToMap(environment,
        pixel.at("pixel_x").get<double>(), pixel.at("pixel_y").get<double>());
      poses.push_back({{"x", point.x}, {"y", point.y}, {"yaw", yaw}});
    } catch (const std::invalid_argument & error) {
      return ToolResult::failure("INVALID_ARGUMENTS", error.what());
    }
  }
  Json goal{{"frame_id", environment.frame_id}};
  if (through_pixels) {goal["poses"] = std::move(poses);}
  else {goal.update(poses.at(0));}
  const auto started = through_pixels ? start_through_poses_navigation(goal) : start_navigation(goal);
  // 每批只提交一次 Action，全部像素转换成功之后才发送，接受之后才消费快照。
  if (!started.payload.at("ok").get<bool>()) {
    auto result = ToolResult::failure(
      started.payload.at("code").get<std::string>(), started.payload.at("message").get<std::string>());
    result.payload["data"]["environment_id"] = id;
    return result;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = std::find_if(environments_.begin(), environments_.end(),
      [&](const auto & item) {return item.id == id;});
    if (found != environments_.end()) {environments_.erase(found);}
  }
  Json data{{"environment_id", id}, {"task_id", started.payload.at("data").at("task_id")},
    {"status", started.payload.at("data").at("status")}};
  if (through_pixels) {
    data["frame_id"] = environment.frame_id;
    data["goals"] = std::move(goal["poses"]);
  } else {data["goal"] = std::move(goal);}
  return ToolResult::success(std::move(data));
}

void NavigateToPixelStartNavigation::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult NavigateToPixelStartNavigation::execute(const Json & arguments)
{
  return navigation_->navigate_to_pixel_start_navigation(arguments);
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::NavigateToPixelStartNavigation, vln_agent::ToolPluginBase)
