#include "vln_agent/plugins/tools/navigation/lookaround.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"
#include "vln_agent/utils/base64.hpp"
#include "vln_agent/utils/time.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace vln_agent
{
ToolResult NavigationRuntime::lookaround(RgbdRuntime & rgbd, const Json & settings)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");}
    if (task_ || lookaround_running_) {return ToolResult::failure("NAVIGATION_BUSY", "已有活动导航任务");}
    lookaround_running_ = true;
  }
  const auto release = [this] {
      std::lock_guard<std::mutex> lock(mutex_);
      lookaround_running_ = false;
    };
  try {
    const auto robot = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(costmap_timeout_sec_));
    const double initial_yaw = tf2::getYaw(robot.transform.rotation);
    const auto turn = [&](int step) {
        Action::Goal goal;
        goal.pose.header.frame_id = map_frame_;
        goal.pose.header.stamp = node_.now();
        goal.pose.pose.position.x = robot.transform.translation.x;
        goal.pose.pose.position.y = robot.transform.translation.y;
        const double yaw = initial_yaw + step * M_PI / 4;
        goal.pose.pose.orientation.z = std::sin(yaw / 2);
        goal.pose.pose.orientation.w = std::cos(yaw / 2);
        return sendGoal(goal, true);
      };
    auto result = rgbd.capture_surround(turn, settings.value("pause_sec", 0.3),
      settings.value("tile_size", 512), settings.value("gap_size", 12), initial_yaw);
    release();
    return result;
  } catch (...) {release(); throw;}
}

ToolResult RgbdRuntime::capture_surround(const std::function<ToolResult(int)> & turn,
  double pause_sec, int tile_size, int gap_size, double yaw)
{
  auto surround = std::make_shared<Surround>();
  for (int i = 0; i < 8; ++i) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (condition_.wait_for(lock, std::chrono::duration<double>(pause_sec), [this] {return closed_;})) {
        return ToolResult::failure("TOOL_MANAGER_CLOSED", "环视采集期间资源关闭");
      }
    }
    auto captured = capturePair({{"mode", "latest"}, {"detail", 0}}, surround->pairs[i]);
    if (!captured.payload.at("ok").get<bool>()) {return captured;}
    auto turned = turn(i + 1);
    if (!turned.payload.at("ok").get<bool>()) {return turned;}
  }
  // 与原行为树一致：第一张在上方，其余按逆时针排布，保留实际缩放和居中留黑。
  const std::array<cv::Point, 8> cells{{{1, 0}, {0, 0}, {0, 1}, {0, 2},
    {1, 2}, {2, 2}, {2, 1}, {2, 0}}};
  const int stride = tile_size + gap_size;
  cv::Mat mosaic(3 * tile_size + 2 * gap_size, 3 * tile_size + 2 * gap_size,
    CV_8UC3, cv::Scalar(180, 180, 180));
  Json views = Json::array();
  for (int i = 0; i < 8; ++i) {
    const auto & color = surround->pairs[i]->color;
    const auto cell = cells[i];
    const cv::Rect tile(cell.x * stride, cell.y * stride, tile_size, tile_size);
    mosaic(tile).setTo(cv::Scalar(12, 12, 12));
    const double scale = std::min({1.0, double(tile_size) / color.cols, double(tile_size) / color.rows});
    const cv::Size size(std::max(1, int(std::lround(color.cols * scale))),
      std::max(1, int(std::lround(color.rows * scale))));
    const cv::Rect rect(tile.x + (tile_size - size.width) / 2,
      tile.y + (tile_size - size.height) / 2, size.width, size.height);
    cv::resize(color, mosaic(rect), size, 0, 0, cv::INTER_AREA);
    surround->image_rects[i] = rect;
    cv::putText(mosaic, std::to_string(i + 1), {tile.x + 4, tile.y + std::max(12, tile_size / 12)},
      cv::FONT_HERSHEY_SIMPLEX, std::min(0.6, tile_size / 200.0), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    views.push_back({{"index", i}, {"image_rect", {{"x", rect.x}, {"y", rect.y},
        {"width", rect.width}, {"height", rect.height}}}});
  }
  const cv::Rect center(stride, stride, tile_size, tile_size);
  mosaic(center).setTo(cv::Scalar(32, 32, 32));
  cv::putText(mosaic, cv::format("yaw %.1f deg", yaw * 180 / M_PI),
    {center.x + 4, center.y + tile_size / 2}, cv::FONT_HERSHEY_SIMPLEX,
    std::min(0.7, tile_size / 220.0), cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
  std::vector<unsigned char> jpeg;
  cv::imencode(".jpg", mosaic, jpeg, {cv::IMWRITE_JPEG_QUALITY, 85});
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "RGB-D 资源已经关闭");}
    surround->id = "lookaround_" + std::to_string(++next_surround_id_);
    surround_ = surround;  // 只有整轮成功才替换缓存；普通 capture_pair 不会淘汰它。
  }
  auto result = ToolResult::success({{"lookaround_id", surround->id}, {"views", std::move(views)}});
  result.payload["images"] = Json::array({{{"time_stamp", utils::utcNow()}, {"metadata", {
        {"layout", "surround_3x3"}, {"direction", "counterclockwise"}, {"view_count", 8},
        {"width", mosaic.cols}, {"height", mosaic.rows}}}}});
  result.images.push_back({{"type", "input_image"},
      {"image_url", "data:image/jpeg;base64," + utils::encodeBase64(jpeg)}, {"detail", "high"}});
  return result;
}

void Lookaround::initialize(const ToolContext & context, const Json & settings)
{
  resources_ = context.resources;
  settings_ = settings;
  const double pause = settings.value("pause_sec", 0.3);
  if (!std::isfinite(pause) || pause < 0 || settings.value("tile_size", 512) <= 0 ||
    settings.value("gap_size", 12) < 0) {throw std::invalid_argument("环视停顿与间隔须非负，网格尺寸须为正");}
  rgbd_resource_ = settings.value("rgbd_resource", std::string("rgbd/rgbd"));
  navigation_ = resources_->getOrCreate<NavigationRuntime>(context.group_id + "/navigation", [&] {
      return std::make_shared<NavigationRuntime>(context.node, context.group_settings);
    });
}
ToolResult Lookaround::execute(const Json &)
{
  return navigation_->lookaround(*resources_->get<RgbdRuntime>(rgbd_resource_), settings_);
}
}  // namespace vln_agent
PLUGINLIB_EXPORT_CLASS(vln_agent::Lookaround, vln_agent::ToolPluginBase)
