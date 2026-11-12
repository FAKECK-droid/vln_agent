#include "vln_agent/plugins/tools/rgbd/utils.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <vector>

using Clock = std::chrono::steady_clock;
/** @brief 输出纯投影采样统计；不包含 SAM、压缩图解码、TF 查询或 ROS 调度。 */
template<class Function>
void benchmark(const char * name, Function function)
{
  for (int i = 0; i < 10; ++i) {function();}
  std::vector<double> times;
  times.reserve(100);
  std::size_t count = 0;
  for (int i = 0; i < 100; ++i) {
    const auto start = Clock::now();
    count = function();
    times.push_back(std::chrono::duration<double, std::milli>(Clock::now() - start).count());
  }
  const double mean = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
  std::sort(times.begin(), times.end());
  std::cout << name << " count=" << count << " mean_ms=" << mean <<
    " p50_ms=" << times[50] << " p95_ms=" << times[94] << std::endl;
}

int main()
{
  std::cout << "OpenCV_threads=" << cv::getNumThreads() << " samples=100 warmup=10" << std::endl;
  for (const cv::Size size : {cv::Size(640, 360), cv::Size(1280, 720), cv::Size(1920, 1080)}) {
    sensor_msgs::msg::CameraInfo info;
    info.width = size.width; info.height = size.height;
    info.k = {size.width * 0.7, 0, size.width * 0.5,
      0, size.width * 0.7, size.height * 0.5, 0, 0, 1};
    info.d = {0.04, -0.01, 0.001, 0.001, 0}; info.distortion_model = "plumb_bob";
    vln_agent::rgbd::DepthImage depth;
    depth.pixels = cv::Mat(size, CV_16UC1);
    for (int v = 0; v < size.height; ++v) {
      auto * row = depth.pixels.ptr<std::uint16_t>(v);
      for (int u = 0; u < size.width; ++u) {row[u] = (u + v) % 20 == 0 ? 0 : 1000 + (u + v) % 3000;}
    }
    const auto start = Clock::now();
    const auto rays = vln_agent::rgbd::projectionRays(info);
    std::cout << "resolution=" << size.width << "x" << size.height <<
      " ray_table_build_ms=" << std::chrono::duration<double, std::milli>(Clock::now() - start).count() <<
      " ray_table_bytes=" << rays.total() * rays.elemSize() << std::endl;
    cv::Mat mask(size, CV_8UC1, cv::Scalar(1));
    benchmark("mask_full", [&] {return vln_agent::rgbd::meanMaskedDepth(depth, rays, mask).count;});
    mask.setTo(0);
    mask(cv::Rect(size.width / 4, size.height / 4, size.width / 2, size.height / 2)).setTo(1);
    benchmark("mask_25pct", [&] {return vln_agent::rgbd::meanMaskedDepth(depth, rays, mask).count;});
    std::vector<cv::Point> points;
    for (int i = 0; i < 63; ++i) {points.emplace_back((i * 83 + 1) % size.width, (i * 53 + 1) % size.height);}
    benchmark("positive_points_63", [&] {
      return vln_agent::rgbd::meanPointDepth(depth, info, points, size).count;
    });
  }
}
