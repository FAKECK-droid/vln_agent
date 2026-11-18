#include "vln_agent/plugins/tools/rgbd/capture_pair_sequence.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{
ToolResult RgbdRuntime::capture_pair_sequence(double image_scale, std::uint32_t limit)
{
  // 批次串行；单张取图、TF 回调及投影不等待这把锁。
  std::lock_guard<std::mutex> batch_lock(batch_mutex_);
  if (max_pairs_ < 2) {
    return ToolResult::failure("INVALID_CONFIGURATION", "图像序列要求 rgbd.max_pairs 至少为 2");
  }
  auto request = std::make_shared<GetImageBatch::Request>();
  request->stream_id = batch_stream_id_;
  request->after_seq = batch_after_seq_;
  request->max_pairs = std::min<std::size_t>(limit, max_pairs_);
  const auto timeout = std::chrono::duration<double>(timeout_sec_);
  if (!batch_client_->wait_for_service(timeout)) {
    return ToolResult::failure("IMAGE_SERVICE_UNAVAILABLE", "批量取图服务不可用");
  }
  auto future = batch_client_->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    batch_client_->remove_pending_request(future);
    return ToolResult::failure("IMAGE_SERVICE_TIMEOUT", "等待批量取图服务响应超时");
  }
  auto response = future.get();
  const bool latest_available = response->latest_index >= 0;
  const auto latest_index = static_cast<std::size_t>(response->latest_index);
  const auto latest_id = latest_available ? response->pairs.at(latest_index).pair_id : 0;
  const bool latest_is_new = latest_available &&
    (response->stream_id != batch_stream_id_ || latest_id != batch_latest_id_);
  auto result = ToolResult::success({
    {"latest_available", latest_available}, {"latest_is_new", latest_is_new},
    {"latest_index", nullptr}, {"history_enabled", response->history_enabled},
    {"has_more", response->has_more}, {"missing_count", response->missing_count},
    {"cursor_reset", response->cursor_reset}});
  auto & data = result.payload.at("data");
  result.payload["images"] = Json::array();
  result.payload.at("images").get_ref<Json::array_t &>().reserve(response->pairs.size());
  result.images.reserve(response->pairs.size());
  std::vector<std::shared_ptr<Pair>> cached;
  cached.reserve(response->pairs.size());
  for (std::size_t i = 0; i < response->pairs.size(); ++i) {
    auto & input = response->pairs[i];
    auto image = std::make_shared<GetImage::Response>();
    image->pair_id = input.pair_id;
    image->queue_index = -1;
    image->image_age_sec = input.image_age_sec;
    image->color = std::move(input.color);
    image->depth = std::move(input.depth);
    image->stamp_delta_sec = std::abs(
      (rclcpp::Time(image->color.header.stamp) - rclcpp::Time(image->depth.header.stamp)).seconds());
    std::shared_ptr<Pair> pair;
    // 历史查已有 TF，最新帧最多短等一次，避免对每帧累加服务超时。
    const auto prepared = preparePair(std::move(image), pair,
      i == latest_index ? std::min(timeout_sec_, 0.1) : 0.0);
    if (pair->color.empty()) {
      data["unreadable_frames"].push_back({{"source_pair_id", input.pair_id},
        {"history_seq", input.history_seq}});
      continue;
    }
    if (i == latest_index) {data["latest_index"] = result.images.size();}
    appendCaptureImage(result, *pair, image_scale, "auto");
    auto & metadata = result.payload.at("images").back().at("metadata");
    metadata["history_seq"] = input.history_seq;
    metadata["exceeded_max_age"] = input.exceeded_max_age;
    metadata["projectable"] = !pair->id.empty();
    if (pair->id.empty()) {
      metadata["projection_code"] = prepared.payload.at("code");
    } else {
      // 只缓存压缩图。普通正点投影也无需再次解码彩色图。
      pair->color.release();
      cached.push_back(std::move(pair));
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "RGB-D 资源已经关闭");}
    for (auto & pair : cached) {pairs_.push_back(std::move(pair));}
    while (pairs_.size() > max_pairs_) {pairs_.pop_front();}
    data["valid_pair_ids"] = validPairIdsLocked();
  }
  data["count"] = result.images.size();
  batch_stream_id_ = std::move(response->stream_id);
  batch_after_seq_ = response->next_seq;
  batch_latest_id_ = latest_id;
  return result;
}

void CapturePairSequence::initialize(const ToolContext & context, const Json & settings)
{
  image_scale_ = settings.value("image_scale", 0.5);
  if (!std::isfinite(image_scale_) || image_scale_ <= 0) {
    throw std::invalid_argument("capture_pair_sequence.image_scale 必须为有限正数");
  }
  const auto limit = settings.value("max_batch_pairs", 8);
  if (limit < 2 || limit > 32) {
    throw std::invalid_argument("capture_pair_sequence.max_batch_pairs 必须为 2 到 32");
  }
  max_batch_pairs_ = static_cast<std::uint32_t>(limit);
  rgbd_ = context.resources->getOrCreate<RgbdRuntime>(
    context.group_id + "/rgbd", [&] {
      return std::make_shared<RgbdRuntime>(context.node, context.group_settings);
    });
}

ToolResult CapturePairSequence::execute(const Json &)
{
  return rgbd_->capture_pair_sequence(image_scale_, max_batch_pairs_);
}
}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::CapturePairSequence, vln_agent::ToolPluginBase)
