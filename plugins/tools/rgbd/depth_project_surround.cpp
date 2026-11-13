#include "vln_agent/plugins/tools/rgbd/depth_project_surround.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{
ToolResult RgbdRuntime::depth_project_surround(const Json & arguments)
{
  const auto id = arguments.at("lookaround_id").get<std::string>();
  std::shared_ptr<Surround> surround;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    surround = surround_;
  }
  if (!surround || surround->id != id) {
    return ToolResult::failure("LOOKAROUND_NOT_FOUND", "环视 ID 不存在或已被下一轮替换");
  }
  const auto & instances = arguments.at("instances");
  if (instances.empty()) {return ToolResult::failure("INVALID_ARGUMENTS", "instances 至少包含一个实例");}
  std::array<Json, 8> groups;
  std::array<std::vector<std::size_t>, 8> indices;
  for (auto & group : groups) {group = Json::array();}
  Json rows = Json::array();
  rows.get_ref<Json::array_t &>().resize(instances.size());
  for (std::size_t i = 0; i < instances.size(); ++i) {
    try {
      auto prompt = instances[i];
      const auto & points = prompt.at("points");
      const auto & box = prompt.at("box");
      cv::Point anchor;
      if (!points.empty()) {anchor = {points[0].at("x").get<int>(), points[0].at("y").get<int>()};}
      else if (!box.is_null()) {
        anchor = {(box.at("x_min").get<int>() + box.at("x_max").get<int>()) / 2,
          (box.at("y_min").get<int>() + box.at("y_max").get<int>()) / 2};
      } else {throw std::invalid_argument("实例需要框或提示点");}
      const auto found = std::find_if(surround->image_rects.begin(), surround->image_rects.end(),
        [&](const auto & rect) {return rect.contains(anchor);});
      if (found == surround->image_rects.end()) {throw std::invalid_argument("提示须位于环视图的实际图像区域");}
      const auto view = std::size_t(found - surround->image_rects.begin());
      const auto & rect = *found;
      const auto size = surround->pairs[view]->image_size;
      for (auto & point : prompt["points"]) {
        const cv::Point input(point.at("x").get<int>(), point.at("y").get<int>());
        if (!rect.contains(input)) {throw std::invalid_argument("同一实例的提示不能跨越多个视角");}
        const auto original = rgbd::toDepthPixel(input.x - rect.x, input.y - rect.y, rect.size(), size);
        point["x"] = original.x; point["y"] = original.y;
      }
      if (!box.is_null()) {
        const int x0 = box.at("x_min"), y0 = box.at("y_min"), x1 = box.at("x_max"), y1 = box.at("y_max");
        if (!rect.contains({x0, y0}) || !rect.contains({x1 - 1, y1 - 1}) || x1 <= x0 || y1 <= y0) {
          throw std::invalid_argument("实例框须完整位于同一视角内");
        }
        prompt["box"] = {{"x_min", int(std::lround(double(x0 - rect.x) * size.width / rect.width))},
          {"y_min", int(std::lround(double(y0 - rect.y) * size.height / rect.height))},
          {"x_max", int(std::lround(double(x1 - rect.x) * size.width / rect.width))},
          {"y_max", int(std::lround(double(y1 - rect.y) * size.height / rect.height))}};
      }
      groups[view].push_back(std::move(prompt));
      indices[view].push_back(i);
    } catch (const std::exception & error) {
      rows[i] = {{"index", i}, {"ok", false}, {"code", "INVALID_PROMPT"}, {"message", error.what()}};
    }
  }
  std::size_t successes = 0;
  for (std::size_t view = 0; view < 8; ++view) {
    if (groups[view].empty()) {continue;}
    auto input = arguments;
    input["instances"] = std::move(groups[view]);
    auto projected = projectDepth(surround->pairs[view], input);
    for (std::size_t j = 0; j < indices[view].size(); ++j) {
      auto row = projected.payload.at("data").contains("instances") ?
        projected.payload.at("data").at("instances").at(j) :
        Json{{"ok", false}, {"code", projected.payload.at("code")}, {"message", projected.payload.at("message")}};
      row["index"] = indices[view][j]; row["view_index"] = view;
      if (row.at("ok").get<bool>()) {++successes;}
      rows[indices[view][j]] = std::move(row);
    }
  }
  auto result = successes ? ToolResult::success(Json::object()) :
    ToolResult::failure(rows.front().at("code").get<std::string>(), rows.front().at("message").get<std::string>());
  result.payload["data"] = {{"lookaround_id", id}, {"frame_id", arguments.at("target_frame")},
    {"use_segmentation", arguments.at("use_segmentation")}, {"success_count", successes}, {"instances", std::move(rows)}};
  return result;
}

void DepthProjectSurround::initialize(const ToolContext & context, const Json &)
{
  rgbd_ = context.resources->getOrCreate<RgbdRuntime>(context.group_id + "/rgbd", [&] {
      return std::make_shared<RgbdRuntime>(context.node, context.group_settings);
    });
}
ToolResult DepthProjectSurround::execute(const Json & arguments)
{
  return rgbd_->depth_project_surround(arguments);
}
}  // namespace vln_agent
PLUGINLIB_EXPORT_CLASS(vln_agent::DepthProjectSurround, vln_agent::ToolPluginBase)
