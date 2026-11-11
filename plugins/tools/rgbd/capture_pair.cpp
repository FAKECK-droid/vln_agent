#include "vln_agent/plugins/tools/rgbd/capture_pair.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <future>
#include <map>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/buffer.h>

#include "vln_agent/utils/base64.hpp"
#include "vln_agent/utils/time.hpp"

namespace vln_agent
{
ToolResult RgbdRuntime::capture_pair(const Json & arguments, double image_scale)
{
  static const std::map<std::string, std::uint8_t> operations{
    {"latest", GetImage::Request::LATEST_PAIR},
    {"index", GetImage::Request::QUEUE_PAIR},
    {"age", GetImage::Request::QUEUE_PAIR_AT_AGE}};
  auto request = std::make_shared<GetImage::Request>();
  request->operation = operations.at(arguments.at("mode").get<std::string>());
  request->detail = arguments.at("detail").get<std::uint32_t>();
  const auto timeout = std::chrono::duration<double>(timeout_sec_);
  if (!client_->wait_for_service(timeout)) {
    return ToolResult::failure("IMAGE_SERVICE_UNAVAILABLE", "取图服务不可用");
  }
  auto future = client_->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    client_->remove_pending_request(future);
    return ToolResult::failure("IMAGE_SERVICE_TIMEOUT", "等待取图服务响应超时");
  }
  const auto response = future.get();
  if (!response->success || !response->has_color || !response->has_depth) {
    return ToolResult::failure("IMAGE_PAIR_UNAVAILABLE", response->message);
  }
  auto pair = std::make_shared<Pair>();
  pair->images = response;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pair->info = info_;
  }
  if (!pair->info || pair->info->k[0] <= 0 || pair->info->k[4] <= 0) {
    return ToolResult::failure("CAMERA_INFO_UNAVAILABLE", "尚未收到有效的彩色 CameraInfo");
  }
  const auto & info = *pair->info;
  const auto & color = response->color;
  const auto & depth = response->depth;
  if (depth.header.frame_id != color.header.frame_id || color.header.frame_id != info.header.frame_id) {
    return ToolResult::failure("UNALIGNED_IMAGE_PAIR", "深度必须对齐到彩色光学系，并使用对应 CameraInfo");
  }
  if (info.distortion_model != "plumb_bob" && info.distortion_model != "rational_polynomial" &&
    info.distortion_model != "equidistant" && !info.distortion_model.empty())
  {
    return ToolResult::failure("UNSUPPORTED_CAMERA_MODEL", "不支持的畸变模型: " + info.distortion_model);
  }
  const bool png = color.format.find("png") != std::string::npos;
  std::vector<unsigned char> scaled_color;
  const auto * color_bytes = &color.data;
  try {
    const auto original = cv::imdecode(color.data, cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION);
    if (original.empty() || original.cols != static_cast<int>(info.width) ||
      original.rows != static_cast<int>(info.height))
    {
      return ToolResult::failure("UNALIGNED_IMAGE_PAIR", "彩色图分辨率必须与 CameraInfo 一致");
    }
    pair->image_size = original.size();
    if (image_scale != 1.0) {
      pair->image_size = {
        std::max(1, static_cast<int>(std::lround(original.cols * image_scale))),
        std::max(1, static_cast<int>(std::lround(original.rows * image_scale)))};
      cv::Mat resized;
      cv::resize(original, resized, pair->image_size, 0, 0,
        image_scale < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
      if (!cv::imencode(png ? ".png" : ".jpg", resized, scaled_color)) {
        return ToolResult::failure("IMAGE_PAIR_UNAVAILABLE", "彩色图缩放编码失败");
      }
      color_bytes = &scaled_color;
    }
  } catch (const cv::Exception & error) {
    return ToolResult::failure("IMAGE_PAIR_UNAVAILABLE", error.what());
  }
  if (depth.header.stamp.sec == 0 && depth.header.stamp.nanosec == 0) {
    return ToolResult::failure("TF_UNAVAILABLE", "深度图缺少采样时间，无法保存采样时的相机位姿");
  }
  try {
    pair->fixed_from_camera = tf_buffer_->lookupTransform(
      fixed_frame_, info.header.frame_id, rclcpp::Time(depth.header.stamp),
      rclcpp::Duration::from_seconds(timeout_sec_));
  } catch (const tf2::TransformException & error) {
    return ToolResult::failure("TF_UNAVAILABLE", error.what());
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "RGB-D 资源已经关闭");}
    pair->id = "pair_" + std::to_string(++next_id_);
    pairs_.push_back(pair);
    while (pairs_.size() > max_pairs_) {pairs_.pop_front();}
  }

  Json metadata{{"frame_id", info.header.frame_id},
    {"width", pair->image_size.width}, {"height", pair->image_size.height},
    {"original_width", info.width}, {"original_height", info.height},
    {"image_pair_id", pair->id}, {"pair_id", response->pair_id},
    {"pixel_origin", "top_left"}, {"fixed_frame", fixed_frame_},
    {"queue_index", response->queue_index}, {"image_age_sec", response->image_age_sec},
    {"stamp_delta_sec", response->stamp_delta_sec}};
  Json description{{"metadata", std::move(metadata)}};
  if (real_stamps_) {
    const auto utc = [](const builtin_interfaces::msg::Time & stamp) {
        return utils::formatUtcTime(std::chrono::system_clock::time_point(
          std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec)));
      };
    if (color.header.stamp.sec != 0 || color.header.stamp.nanosec != 0) {
      description["time_stamp"] = utc(color.header.stamp);
    }
    if (depth.header.stamp.sec != 0 || depth.header.stamp.nanosec != 0) {
      description["metadata"]["depth_time_stamp"] = utc(depth.header.stamp);
    }
  }
  auto result = ToolResult::success({{"image_pair_id", pair->id}});
  result.payload["images"] = Json::array({std::move(description)});
  const auto mime = png ? "image/png" : "image/jpeg";
  result.images.push_back({{"type", "input_image"},
    {"image_url", std::string("data:") + mime + ";base64," + utils::encodeBase64(*color_bytes)},
    {"detail", "high"}});
  return result;
}

void CapturePair::initialize(const ToolContext & context, const Json & settings)
{
  image_scale_ = settings.value("image_scale", 1.0);
  if (!std::isfinite(image_scale_) || image_scale_ <= 0) {
    throw std::invalid_argument("capture_pair.image_scale 必须为有限正数");
  }
  rgbd_ = context.resources->getOrCreate<RgbdRuntime>(
    context.group_id + "/rgbd", [&] {
      return std::make_shared<RgbdRuntime>(context.node, context.group_settings);
    });
}

ToolResult CapturePair::execute(const Json & arguments)
{
  return rgbd_->capture_pair(arguments, image_scale_);
}
}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::CapturePair, vln_agent::ToolPluginBase)
