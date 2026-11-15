#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "vln_agent/plugins/tools/navigation/utils.hpp"

namespace vln_agent::navigation::test
{
nav2_msgs::msg::Costmap grid(int cells, double resolution)
{
  nav2_msgs::msg::Costmap map;
  map.metadata.size_x = map.metadata.size_y = cells;
  map.metadata.resolution = resolution;
  map.metadata.origin.orientation.w = 1;
  map.data.assign(cells * cells, 0);
  return map;
}

TEST(NavigationImages, MapAxesStayFixedAndRobotMarkerTracksHeading)
{
  auto map = grid(200, 0.1);
  map.data[110 * 200 + 110] = 254;  // 地图点 (11.05, 11.05)，位于机器人右上。
  const cv::Vec3b heading_blue(215, 100, 20);
  for (const double yaw : {0.0, M_PI / 2, -0.73}) {
    Environment env;
    env.robot = {8, 9, yaw};
    env.range_m = 10;
    const auto image = renderSurroundings(map, {0, 0, 0}, env, 512, "global");
    const double x = env.map_rect.x + 255.5, y = env.map_rect.y + 255.5;
    const auto center = pixelToMap(env, x, y);
    EXPECT_NEAR(center.x, 8, 1e-10);
    EXPECT_NEAR(center.y, 9, 1e-10);
    const auto up = pixelToMap(env, x, y - 1 / env.meters_per_pixel);
    EXPECT_NEAR(up.x, 8, 1e-10);
    EXPECT_NEAR(up.y, 10, 1e-10);
    const auto right = pixelToMap(env, x + 1 / env.meters_per_pixel, y);
    EXPECT_NEAR(right.x, 9, 1e-10);
    EXPECT_NEAR(right.y, 9, 1e-10);
    EXPECT_EQ(image.at<cv::Vec3b>(cvRound(y - 2.05 / env.meters_per_pixel),
        cvRound(x + 3.05 / env.meters_per_pixel)), cv::Vec3b(28, 32, 38));
    const double dx = 24 * std::cos(yaw), dy = -24 * std::sin(yaw);
    EXPECT_EQ(image.at<cv::Vec3b>(cvRound(y + dy), cvRound(x + dx)), heading_blue);
    EXPECT_NE(image.at<cv::Vec3b>(cvRound(y - dy), cvRound(x - dx)), heading_blue);
    EXPECT_THROW(pixelToMap(env, 0, 0), std::invalid_argument);
  }
}

TEST(NavigationImages, RotatedGridOriginPreservesKnownObstacleLocation)
{
  auto map = grid(40, 0.1);
  map.data[20 * 40 + 30] = 254;
  Environment env;
  env.robot = {4, 6, 0};
  env.range_m = 8;
  const auto image = renderSurroundings(map, {5, 5, M_PI / 2}, env, 512, "local");
  // cell(30,20) center -> rotated origin -> map(2.95,8.05)
  const double cx = env.map_rect.x + 255.5, cy = env.map_rect.y + 255.5;
  EXPECT_EQ(image.at<cv::Vec3b>(cvRound(cy - 2.05 / env.meters_per_pixel),
      cvRound(cx - 1.05 / env.meters_per_pixel)), cv::Vec3b(28, 32, 38));
}

TEST(NavigationImages, OutsideMapRemainsUnknownAndDownsamplingRetainsThinWalls)
{
  auto map = grid(1000, 0.01);
  for (int y = 0; y < 1000; ++y) {map.data[y * 1000 + 490] = 254;}
  Environment env;
  env.robot = {5, 5, 0.41};
  env.range_m = 20;
  const auto image = renderSurroundings(map, {0, 0, 0}, env, 256, "global");
  EXPECT_EQ(image.at<cv::Vec3b>(env.map_rect.y + 2, env.map_rect.x + 2),
    cv::Vec3b(165, 165, 165));
  cv::Mat blocked;
  cv::inRange(image(env.map_rect), cv::Scalar(28, 32, 38), cv::Scalar(28, 32, 38), blocked);
  EXPECT_GT(cv::countNonZero(blocked), 80);
}

TEST(NavigationImages, WritesMapAlignedPreviewWithCostsLegendAndScale)
{
  auto map = grid(480, 0.05);
  cv::Mat free_mask(480, 480, CV_8UC1, cv::Scalar(0));
  cv::rectangle(free_mask, {210, 0, 60, 480}, 255, -1);
  cv::rectangle(free_mask, {0, 214, 480, 52}, 255, -1);
  cv::rectangle(free_mask, {290, 290, 145, 145}, 255, -1);
  cv::rectangle(free_mask, {320, 260, 38, 48}, 255, -1);
  cv::rectangle(free_mask, {55, 50, 130, 130}, 255, -1);
  cv::rectangle(free_mask, {175, 112, 47, 32}, 255, -1);
  cv::rectangle(free_mask, {350, 340, 30, 38}, 0, -1);
  cv::Mat distance, expanded;
  cv::distanceTransform(free_mask, distance, cv::DIST_L2, 3);
  cv::dilate(free_mask, expanded, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
  for (int y = 0; y < 480; ++y) {
    for (int x = 0; x < 480; ++x) {
      const float meters = distance.at<float>(y, x) * 0.05f;
      map.data[y * 480 + x] = !free_mask.at<unsigned char>(y, x) ?
        (expanded.at<unsigned char>(y, x) ? 254 : 255) :
        meters < 0.20f ? 253 : meters < 0.45f ? 210 : meters < 0.85f ? 80 : 0;
    }
  }
  Environment env;
  env.id = "env_demo_1"; env.frame_id = "map";
  env.robot = {-0.2, 0, M_PI / 6}; env.range_m = 18;
  const auto directory = std::filesystem::path(VLN_TEST_OUTPUT_DIR) / "navigation";
  std::filesystem::create_directories(directory);
  cv::TickMeter timer;
  timer.start();
  const auto image = renderSurroundings(map, {-12, -12, 0}, env, 512, "global");
  std::vector<unsigned char> png;
  ASSERT_TRUE(cv::imencode(".png", image, png, {cv::IMWRITE_PNG_COMPRESSION, 1}));
  timer.stop();
  const auto preview = directory / "surroundings_map_aligned.png";
  ASSERT_TRUE(cv::imwrite(preview.string(), image));
  std::cout << "PREVIEW " << preview <<
    " render_and_png_ms=" << timer.getTimeMilli() << " png_bytes=" << png.size() << std::endl;
}
}  // namespace vln_agent::navigation::test
