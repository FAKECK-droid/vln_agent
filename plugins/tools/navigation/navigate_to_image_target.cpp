#include "vln_agent/plugins/tools/navigation/navigate_to_image_target.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{
void NavigateToImageTarget::initialize(const ToolContext & context, const Json & settings)
{
  resources_ = context.resources;
  rgbd_resource_ = context.group_settings.value("rgbd_resource", std::string("rgbd/rgbd"));
  scoring_ = navigation::GoalScoring::fromSettings(settings);
  navigation_ = resources_->getOrCreate<NavigationRuntime>(
    context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}

ToolResult NavigateToImageTarget::execute(const Json & arguments)
{
  const auto rgbd = resources_->get<RgbdRuntime>(rgbd_resource_);
  Json request{{"image_pair_id", arguments.at("image_pair_id")},
    {"goals", Json::array({{{"pixel_x", arguments.at("pixel_x")},
        {"pixel_y", arguments.at("pixel_y")}, {"stand_off_m", arguments.at("stand_off_m")}}})}};
  auto evaluated = navigation_->evaluateImageGoals(*rgbd, request, scoring_, 8);
  if (!evaluated.payload.at("ok").get<bool>()) {return evaluated;}
  const auto & best = evaluated.payload.at("data").at("best_goal");
  if (best.is_null()) {
    auto failed = ToolResult::failure("NO_REACHABLE_GOAL", "无法选出可用接近位置，原因见 goals 评估明细");
    failed.payload["data"] = std::move(evaluated.payload["data"]);
    return failed;
  }
  auto started = navigation_->start_navigation(best.at("goal"));
  auto & data = started.payload["data"];
  data["image_pair_id"] = arguments.at("image_pair_id");
  data["pixel_x"] = arguments.at("pixel_x"); data["pixel_y"] = arguments.at("pixel_y");
  data["stand_off_m"] = arguments.at("stand_off_m");
  for (const auto & key : {"target", "goal", "path_length_m", "goal_clearance_m", "score"}) {
    data[key] = best.at(key);
  }
  return started;
}
}  // namespace vln_agent
PLUGINLIB_EXPORT_CLASS(vln_agent::NavigateToImageTarget, vln_agent::ToolPluginBase)
