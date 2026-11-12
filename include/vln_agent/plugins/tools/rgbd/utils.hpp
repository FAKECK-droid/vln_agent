#pragma once

#include <array>
#include <vector>

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

/** @brief 判断两次标定是否共享同一像素投影几何。 */
bool sameCalibration(const sensor_msgs::msg::CameraInfo & a,
  const sensor_msgs::msg::CameraInfo & b);
/** @brief 原始网格的去畸变 (x/z,y/z) 射线表，同一标定只需构建一次。 */
cv::Mat projectionRays(const sensor_msgs::msg::CameraInfo & info);
/** @brief 相机光学坐标系中的有效深度均值。 */
struct DepthMean
{
  cv::Vec3d point{0, 0, 0}; ///< x/y/z 均值，单位米；z 也是平均光轴深度。
  std::size_t count{0};    ///< 参与均值的有效像素数。
};
/** @brief 从原始掩码直接累加有效深度，按行并行归约，不创建点云。 */
DepthMean meanMaskedDepth(const DepthImage & depth, const cv::Mat & rays,
  const cv::Mat & mask);
/** @brief 一次去畸变全部正点，按返回图实际尺寸映射原始深度网格并求均值。 */
DepthMean meanPointDepth(const DepthImage & depth,
  const sensor_msgs::msg::CameraInfo & info, const std::vector<cv::Point> & pixels,
  cv::Size input_size);
/** @brief 按 CameraInfo 的 K/D 去畸变并反投影到彩色光学坐标系，单位米。 */
geometry_msgs::msg::Point backProject(
  const sensor_msgs::msg::CameraInfo & info, int u, int v, double depth);
}  // namespace vln_agent::rgbd
