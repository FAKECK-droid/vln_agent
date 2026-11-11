#include "vln_agent/plugins/tools/rgbd/utils.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

namespace vln_agent::rgbd
{
DepthImage decodeDepth(const sensor_msgs::msg::CompressedImage & image)
{
  // ROS ConfigHeader 是 int32 format + 两个 float32，PNG 紧随其后。
  constexpr std::size_t header_size = sizeof(std::int32_t) + 2 * sizeof(float);
  const auto encoding = image.format.substr(0, image.format.find(';'));
  if ((encoding != "16UC1" && encoding != "32FC1") ||
    image.format.find("compressedDepth") == std::string::npos ||
    image.format.find("rvl") != std::string::npos || image.data.size() <= header_size)
  {
    throw std::invalid_argument("需要 16UC1 或 32FC1 compressedDepth PNG");
  }
  DepthImage result;
  result.inverse = encoding == "32FC1";
  std::memcpy(result.parameters.data(), image.data.data() + sizeof(std::int32_t),
    sizeof(result.parameters));
  const cv::Mat encoded(1, static_cast<int>(image.data.size() - header_size), CV_8U,
    const_cast<unsigned char *>(image.data.data() + header_size));
  result.pixels = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
  if (result.pixels.empty() || result.pixels.type() != CV_16UC1) {
    throw std::invalid_argument("深度 PNG 必须能解码为单通道 uint16");
  }
  return result;
}

double depthAt(const DepthImage & image, int u, int v)
{
  if (u < 0 || v < 0 || u >= image.pixels.cols || v >= image.pixels.rows) {
    throw std::out_of_range("像素坐标超出原始图片范围");
  }
  const auto value = image.pixels.at<std::uint16_t>(v, u);
  const double depth = image.inverse ?
    image.parameters[0] / (static_cast<double>(value) - image.parameters[1]) : value * 0.001;
  if (value == 0 || !std::isfinite(depth) || depth <= 0) {
    throw std::invalid_argument("该像素没有有效深度");
  }
  return depth;
}

cv::Point toDepthPixel(int u, int v, cv::Size image_size, cv::Size depth_size)
{
  if (u < 0 || v < 0 || u >= image_size.width || v >= image_size.height) {
    throw std::out_of_range("像素坐标超出返回彩色图范围");
  }
  // resize 的像素中心为 (u + 0.5) * 原宽 / 新宽 - 0.5；最近像素取整后如下。
  const double scale_x = depth_size.width / static_cast<double>(image_size.width);
  const double scale_y = depth_size.height / static_cast<double>(image_size.height);
  return {static_cast<int>(std::floor((u + 0.5) * scale_x)),
    static_cast<int>(std::floor((v + 0.5) * scale_y))};
}

geometry_msgs::msg::Point backProject(
  const sensor_msgs::msg::CameraInfo & info, int u, int v, double depth)
{
  const cv::Mat camera(3, 3, CV_64F, const_cast<double *>(info.k.data()));
  const std::vector<cv::Point2d> pixel{{static_cast<double>(u), static_cast<double>(v)}};
  std::vector<cv::Point2d> ray;
  if (info.distortion_model == "equidistant") {
    cv::fisheye::undistortPoints(pixel, ray, camera, info.d);
  } else {
    cv::undistortPoints(pixel, ray, camera, info.d);
  }
  geometry_msgs::msg::Point point;
  point.x = ray[0].x * depth;
  point.y = ray[0].y * depth;
  point.z = depth;
  return point;
}
}  // namespace vln_agent::rgbd
