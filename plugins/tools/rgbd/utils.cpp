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

namespace
{
/** @brief 热路径使用零值表示无效深度，避免逐像素构造异常。 */
inline double depthValue(const DepthImage & depth, std::uint16_t value)
{
  if (!value) {return 0;}
  const double z = depth.inverse ?
    depth.parameters[0] / (static_cast<double>(value) - depth.parameters[1]) : value * 0.001;
  return std::isfinite(z) && z > 0 ? z : 0;
}

cv::Mat undistort(const sensor_msgs::msg::CameraInfo & info, const cv::Mat & pixels)
{
  const cv::Mat camera(3, 3, CV_64F, const_cast<double *>(info.k.data()));
  cv::Mat rays;
  if (info.distortion_model == "equidistant") {
    cv::fisheye::undistortPoints(pixels, rays, camera, info.d);
  } else {
    cv::undistortPoints(pixels, rays, camera, info.d);
  }
  return rays;
}
}

bool sameCalibration(const sensor_msgs::msg::CameraInfo & a,
  const sensor_msgs::msg::CameraInfo & b)
{
  return a.width == b.width && a.height == b.height && a.k == b.k && a.d == b.d &&
         a.distortion_model == b.distortion_model;
}

cv::Mat projectionRays(const sensor_msgs::msg::CameraInfo & info)
{
  cv::Mat pixels(info.height, info.width, CV_64FC2);
  for (int v = 0; v < pixels.rows; ++v) {
    auto * row = pixels.ptr<cv::Vec2d>(v);
    for (int u = 0; u < pixels.cols; ++u) {row[u] = {double(u), double(v)};}
  }
  return undistort(info, pixels.reshape(2, pixels.rows * pixels.cols)).reshape(2, info.height);
}

DepthMean meanMaskedDepth(const DepthImage & depth, const cv::Mat & rays, const cv::Mat & mask)
{
  std::vector<cv::Vec4d> sums(depth.pixels.rows, cv::Vec4d(0, 0, 0, 0));
  cv::parallel_for_(cv::Range(0, depth.pixels.rows), [&](const cv::Range & range) {
    for (int v = range.start; v < range.end; ++v) {
      const auto * selected = mask.ptr<std::uint8_t>(v);
      const auto * raw = depth.pixels.ptr<std::uint16_t>(v);
      const auto * ray = rays.ptr<cv::Vec2d>(v);
      auto & sum = sums[v];
      for (int u = 0; u < depth.pixels.cols; ++u) {
        if (!selected[u]) {continue;}
        const double z = depthValue(depth, raw[u]);
        if (!z) {continue;}
        sum[0] += ray[u][0] * z; sum[1] += ray[u][1] * z;
        sum[2] += z; sum[3] += 1;
      }
    }
  });
  cv::Vec4d total(0, 0, 0, 0);
  for (const auto & sum : sums) {total += sum;}
  if (total[3] == 0) {return {};}
  return {{total[0] / total[3], total[1] / total[3], total[2] / total[3]},
    static_cast<std::size_t>(total[3])};
}

DepthMean meanPointDepth(const DepthImage & depth, const sensor_msgs::msg::CameraInfo & info,
  const std::vector<cv::Point> & pixels, cv::Size input_size)
{
  std::vector<cv::Point2d> original;
  std::vector<double> values;
  original.reserve(pixels.size()); values.reserve(pixels.size());
  for (const auto & point : pixels) {
    const auto raw = toDepthPixel(point.x, point.y, input_size, depth.pixels.size());
    const double z = depthValue(depth, depth.pixels.at<std::uint16_t>(raw));
    if (!z) {continue;}
    original.emplace_back(raw.x, raw.y); values.push_back(z);
  }
  if (original.empty()) {return {};}
  const auto rays = undistort(info, cv::Mat(original));
  cv::Vec3d sum(0, 0, 0);
  for (std::size_t i = 0; i < values.size(); ++i) {
    const auto & ray = rays.at<cv::Vec2d>(static_cast<int>(i));
    sum += cv::Vec3d(ray[0] * values[i], ray[1] * values[i], values[i]);
  }
  return {sum / double(values.size()), values.size()};
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
