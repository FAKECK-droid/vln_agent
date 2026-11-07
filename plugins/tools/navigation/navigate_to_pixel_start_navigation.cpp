#include "vln_agent/plugins/tools/navigation/navigate_to_pixel_start_navigation.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include "vln_agent/utils/time.hpp"

namespace vln_agent
{
ToolResult NavigationRuntime::navigate_to_pixel_start_navigation(const Json & arguments)
{
  const auto id = arguments.at("environment_id").get<std::string>();
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
  const auto point = navigation::pixelToMap(environment,
    arguments.at("pixel_x").get<double>(), arguments.at("pixel_y").get<double>());
  const double image_yaw = arguments.at("yaw").get<double>();
  const double yaw = environment.robot.yaw + image_yaw;
  const Json goal{{"frame_id", environment.frame_id}, {"x", point.x}, {"y", point.y}, {"yaw", yaw}};
  const auto started = start_navigation(goal);
  // 仅产生本次像素工具的输出，不再提交一条内部 start_navigation 的工具结果。
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
  return ToolResult::success({{"environment_id", id},
    {"task_id", started.payload.at("data").at("task_id")},
    {"status", started.payload.at("data").at("status")}, {"goal", goal}});
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
