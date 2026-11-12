#include "vln_agent/plugins/tools/rgbd/get_depth_points.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{
void GetDepthPoints::initialize(const ToolContext & context, const Json &)
{
  rgbd_ = context.resources->getOrCreate<RgbdRuntime>(
    context.group_id + "/rgbd", [&] {
      return std::make_shared<RgbdRuntime>(context.node, context.group_settings);
    });
}

ToolResult GetDepthPoints::execute(const Json & arguments)
{
  const auto & inputs = arguments.at("pixels");
  if (inputs.empty() || inputs.size() > 64) {
    return ToolResult::failure("INVALID_ARGUMENTS", "每次查询 1 到 64 个像素");
  }
  const auto id = arguments.at("image_pair_id").get<std::string>();
  const auto frame = arguments.at("target_frame").get<std::string>();
  std::vector<cv::Point> pixels;
  pixels.reserve(inputs.size());
  for (const auto & input : inputs) {
    pixels.emplace_back(input.at("pixel_x").get<int>(), input.at("pixel_y").get<int>());
  }
  auto projected = rgbd_->getDepthPoints(id, pixels, frame);
  Json::array_t points;
  points.reserve(projected.size());
  std::size_t success_count = 0;
  for (std::size_t i = 0; i < projected.size(); ++i) {
    auto & payload = projected[i].payload;
    const bool ok = payload.at("ok").get<bool>();
    auto point = std::move(payload.at("data"));
    point.erase("image_pair_id"); point.erase("frame_id");
    point["pixel_x"] = pixels[i].x; point["pixel_y"] = pixels[i].y;
    point["ok"] = ok;
    if (ok) {++success_count;}
    else {
      point["code"] = std::move(payload.at("code"));
      point["message"] = std::move(payload.at("message"));
    }
    points.push_back(std::move(point));
  }
  auto result = success_count > 0 ? ToolResult::success() :
    ToolResult::failure(points.front().at("code").get<std::string>(),
      points.front().at("message").get<std::string>());
  Json data{{"image_pair_id", id}, {"frame_id", frame}, {"success_count", success_count}};
  data["points"] = std::move(points);
  result.payload["data"] = std::move(data);
  return result;
}
}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::GetDepthPoints, vln_agent::ToolPluginBase)
