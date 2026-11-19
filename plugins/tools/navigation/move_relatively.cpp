#include "vln_agent/plugins/tools/navigation/move_relatively.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"

#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{
ToolResult NavigationRuntime::withRelativeMotion(const std::function<ToolResult()> & motion)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");}
    if (task_ || pending_goals_ != 0 || lookaround_running_ || relative_motion_running_) {
      return ToolResult::failure("NAVIGATION_BUSY", "请等待或取消当前导航动作");
    }
    relative_motion_running_ = true;
  }
  struct ReleaseMotion
  {
    std::mutex & mutex;
    bool & running;
    ~ReleaseMotion()
    {
      std::lock_guard<std::mutex> lock(mutex);
      running = false;
    }
  } release{mutex_, relative_motion_running_};
  return motion();
}

ToolResult NavigationRuntime::move_relatively(const Json & arguments)
{
  const double distance = arguments.at("distance_m").get<double>();
  const double yaw_deg = arguments.at("yaw_deg").get<double>();
  if (!std::isfinite(distance) || std::abs(distance) > 5 ||
    !std::isfinite(yaw_deg) || std::abs(yaw_deg) > 360)
  {
    return ToolResult::failure("INVALID_ARGUMENTS", "距离须在 ±5 米内，转角须在 ±360 度内");
  }
  return withRelativeMotion([&] {
      auto result = ToolResult::success({{"status", "succeeded"}});
      if (distance != 0) {result = driveRelative(distance);}
      const bool translation_completed = result.payload.at("ok").get<bool>();
      if (translation_completed && yaw_deg != 0) {
        result = spinRelative(yaw_deg * std::acos(-1.0) / 180.0);
      }
      auto & data = result.payload["data"];
      data["distance_m"] = distance;
      data["yaw_deg"] = yaw_deg;
      data["translation_completed"] = translation_completed;
      data["rotation_completed"] = yaw_deg == 0 || result.payload.at("ok").get<bool>();
      return result;
    });
}

void MoveRelatively::initialize(const ToolContext & context, const Json &)
{
  navigation_ = context.resources->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult MoveRelatively::execute(const Json & arguments)
{
  return navigation_->move_relatively(arguments);
}
}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::MoveRelatively, vln_agent::ToolPluginBase)
