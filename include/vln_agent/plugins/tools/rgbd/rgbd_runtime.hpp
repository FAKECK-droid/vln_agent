#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rgbd_image_queue/srv/get_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "vln_agent/plugins/tools/rgbd/utils.hpp"
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace tf2_ros {class Buffer; class TransformListener;}

namespace vln_agent
{
class ToolCallbacks;

/** @brief RGB-D 工具共享的取图客户端、标定、TF 和有界图像对缓存。 */
class RgbdRuntime : public SharedResource
{
public:
  /** @brief 创建专用回调线程；settings 来自 rgbd 工具组。 */
  RgbdRuntime(rclcpp::Node & node, const Json & settings);
  ~RgbdRuntime() override;

  /**
   * @brief 保存原始图像对及采样时变换，返回按比例缩放的彩色图。
   * @param arguments 取图模式和索引或年龄。
   * @param image_scale 返回彩色图的缩放因子，1.0 保持原尺寸。
   */
  ToolResult capture_pair(const Json & arguments, double image_scale);
  /** @brief 按缓存 ID/像素反投影，并转换到指定坐标系当前的位置。 */
  ToolResult get_depth_point(const Json & arguments);
  /**
   * @brief 固定同一个 pair 后批量投影，复用深度解码和目标坐标变换。
   * @param id capture_pair 返回的图像对 ID。
   * @param pixels 返回彩色图上的整数像素。
   * @param target 输出坐标系；与 get_depth_point 使用相同时间约定。
   * @return 按输入顺序排列的逐点结果，单点无深度不会丢弃其余结果。
   */
  std::vector<ToolResult> getDepthPoints(
    const std::string & id, const std::vector<cv::Point> & pixels, const std::string & target);
  /** @brief 停止自身回调，释放未决请求、标定与图像缓存。 */
  void shutdown() noexcept override;

private:
  using GetImage = rgbd_image_queue::srv::GetImage;
  struct Pair
  {
    std::string id;                         ///< 本资源内的图像对 ID，与服务端 pair_id 分开。
    GetImage::Response::ConstSharedPtr images;  ///< 同一响应中的彩色、深度压缩图及采样信息。
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;  ///< 捕获时使用的彩色相机标定。
    geometry_msgs::msg::TransformStamped fixed_from_camera;  ///< 深度采样时光学系到固定系的变换。
    cv::Size image_size;                   ///< 模型收到的彩色图实际像素尺寸。
    std::once_flag decode_once;             ///< 同一对深度只解码一次，允许并发查询。
    rgbd::DepthImage depth;                 ///< 首次查询后才占用解码后的像素内存。
  };

  rclcpp::Node & node_;                     ///< 生命周期由宿主节点管理。
  std::unique_ptr<ToolCallbacks> callbacks_;  ///< 两个插件共用的独占回调组与线程。
  rclcpp::Client<GetImage>::SharedPtr client_;  ///< 不通过 Session 或其他组件取图。
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscription_;  ///< 仅保留最新标定。
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;  ///< 按采样时间查询相机位姿的有界 TF 缓存。
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  ///< 复用资源回调线程，不另起线程。
  std::string fixed_frame_;                 ///< 默认 map，固化采样时相机位姿。
  double timeout_sec_{2.0};                 ///< 服务与 TF 等待的时限。
  std::size_t max_pairs_{8};                ///< 仅保留仍可查询的最近 N 对，超出后 FIFO 淘汰。
  bool real_stamps_{true};                  ///< 仿真采样时间不转成 UTC。
  std::mutex mutex_;                       ///< 保护关闭状态、最新标定和缓存索引。
  bool closed_{false};                     ///< 关闭后不再向缓存写入图像。
  std::uint64_t next_id_{0};                ///< 只保存计数，不保存调用历史。
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;  ///< 最新标定；捕获时保存只读引用。
  std::deque<std::shared_ptr<Pair>> pairs_;  ///< 不保存工具输出 JSON/Base64。
};
}  // namespace vln_agent
