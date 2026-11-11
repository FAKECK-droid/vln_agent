#pragma once

#include <array>

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace vln_agent::rgbd
{
/** @brief compressedDepth PNG 的原始 16 位像素及逆深度量化参数。 */
struct DepthImage
{
  cv::Mat pixels;                         ///< 单通道 uint16；0 为无效值。
  bool inverse{false};                   ///< 32FC1 需要按量化参数还原米制深度。
  std::array<float, 2> parameters{};       ///< 逆深度参数 A、B。
};

/** @brief 解码 ROS compressedDepth PNG，支持毫米 16UC1 和逆深度 32FC1。 */
DepthImage decodeDepth(const sensor_msgs::msg::CompressedImage & image);
/** @brief 读取原始彩色网格上的整数像素，返回米制光轴深度；越界或无深度时抛异常。 */
double depthAt(const DepthImage & image, int u, int v);
/**
 * @brief 按像素中心映射到原始深度网格，取最近整数像素；越界时抛异常。
 * @param u 返回彩色图的横坐标，左上角为 0，向右增大。
 * @param v 返回彩色图的纵坐标，向下增大。
 * @param image_size 返回彩色图的实际尺寸，包含缩放后的整数取整。
 * @param depth_size 保留的原始对齐深度尺寸。
 * @return 用于深度采样和 CameraInfo 反投影的原始像素，等距时选右/下像素。
 */
cv::Point toDepthPixel(int u, int v, cv::Size image_size, cv::Size depth_size);

/** @brief 按 CameraInfo 的 K/D 去畸变并反投影到彩色光学坐标系，单位米。 */
geometry_msgs::msg::Point backProject(
  const sensor_msgs::msg::CameraInfo & info, int u, int v, double depth);
}  // namespace vln_agent::rgbd
