#include "vln_agent/plugins/tools/capture_image/capture_image.hpp"

#include <chrono>
#include <cstdint>
#include <future>
#include <map>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "vln_agent/tools/ros_callbacks.hpp"
#include "vln_agent/utils/base64.hpp"
#include "vln_agent/utils/time.hpp"

namespace vln_agent
{
CaptureImage::~CaptureImage()
{
  shutdown();
}

void CaptureImage::initialize(const ToolContext & context, const Json & settings)
{
  bool simulated = false;
  context.node.get_parameter_or("use_sim_time", simulated, false);
  real_stamps_ = !simulated;
  timeout_sec_ = settings.value("timeout_sec", 2.0);
  callbacks_ = std::make_unique<ToolCallbacks>(context.node);
  client_ = context.node.create_client<GetImage>(
    context.group_settings.at("service_name").get<std::string>(),
    rmw_qos_profile_services_default, callbacks_->group());
}

ToolResult CaptureImage::execute(const Json & arguments)
{
  static const std::map<std::string, std::uint8_t> operations{
    {"latest", GetImage::Request::LATEST_COLOR},
    {"index", GetImage::Request::QUEUE_COLOR},
    {"age", GetImage::Request::QUEUE_COLOR_AT_AGE}};
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
  if (!response->success) {
    return ToolResult::failure("IMAGE_UNAVAILABLE", response->message);
  }

  const auto & image = response->color;
  Json payload{{"ok", true}, {"time_stamp", utils::utcNow()},
    {"frame_id", image.header.frame_id}, {"format", image.format},
    {"pair_id", response->pair_id}, {"queue_index", response->queue_index},
    {"queue_size", response->queue_size}, {"image_age_sec", response->image_age_sec},
    {"age_error_sec", response->age_error_sec}, {"exceeded_max_age", response->exceeded_max_age}};
  if (real_stamps_ && (image.header.stamp.sec != 0 || image.header.stamp.nanosec != 0)) {
    payload["image_time_stamp"] = utils::formatUtcTime(std::chrono::system_clock::time_point(
      std::chrono::seconds(image.header.stamp.sec) +
      std::chrono::nanoseconds(image.header.stamp.nanosec)));
  }
  const std::string mime = image.format.find("png") != std::string::npos ?
    "image/png" : "image/jpeg";
  return {payload,
    {{{"type", "input_image"}, {"image_url", "data:" + mime + ";base64," +
        utils::encodeBase64(image.data)}, {"detail", "auto"}}}};
}

void CaptureImage::shutdown() noexcept
{
  if (callbacks_) {callbacks_->shutdown();}
  if (client_) {client_->prune_pending_requests();}
}

}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::CaptureImage, vln_agent::ToolPluginBase)
