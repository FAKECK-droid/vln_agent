#include "vln_agent/plugins/tools/navigation/utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace vln_agent::navigation
{
namespace
{
constexpr int left = 48, top = 72, footer = 112;
const cv::Vec3b free_color{250, 250, 250}, low_color{190, 235, 255};
const cv::Vec3b high_color{60, 165, 245}, blocked_color{28, 32, 38};
const cv::Vec3b unknown_color{165, 165, 165}, trail_color{50, 160, 30};

/** @brief 对一个平面点应用 2×3 仿射变换。 */
cv::Point2d apply(const cv::Matx23d & matrix, double x, double y)
{
  return {matrix(0, 0) * x + matrix(0, 1) * y + matrix(0, 2),
    matrix(1, 0) * x + matrix(1, 1) * y + matrix(1, 2)};
}

/** @brief 合成仿射变换 a(b(point))，不产生中间图片。 */
cv::Matx23d compose(const cv::Matx23d & a, const cv::Matx23d & b)
{
  cv::Matx23d result;
  for (int row = 0; row < 2; ++row) {
    for (int col = 0; col < 3; ++col) {
      result(row, col) = a(row, 0) * b(0, col) + a(row, 1) * b(1, col);
    }
    result(row, 2) += a(row, 2);
  }
  return result;
}

/** @brief 为标题和比例尺生成简短数值标签。 */
std::string number(double value)
{
  std::ostringstream text;
  text << std::setprecision(3) << value;
  return text.str();
}
}  // namespace

cv::Point2d pixelToMap(const Environment & environment, double x, double y)
{
  const auto & rect = environment.map_rect;
  if (!std::isfinite(x) || !std::isfinite(y) ||
    x < rect.x || x > rect.x + rect.width - 1 ||
    y < rect.y || y > rect.y + rect.height - 1)
  {
    throw std::invalid_argument("像素必须位于环境图的 map_rect 内");
  }
  return apply(environment.pixel_to_map, x, y);
}

cv::Mat renderSurroundings(
  const nav2_msgs::msg::Costmap & costmap, const Pose2D & origin,
  Environment & env, int pixels, const std::string & source,
  const std::deque<TrailPoint> & trail)
{
  env.map_rect = {left, top, pixels, pixels};
  env.image_size = {pixels + 2 * left, pixels + top + footer};
  env.meters_per_pixel = env.range_m / pixels;
  const double scale = env.meters_per_pixel;
  const double cx = left + (pixels - 1) * 0.5, cy = top + (pixels - 1) * 0.5;
  // 固定为地图 +X 向右、+Y 向上；机器人的位置只决定取图中心。
  env.pixel_to_map = {
    scale, 0, env.robot.x - scale * cx,
    0, -scale, env.robot.y + scale * cy};

  // 仍需将源栅格（例如 odom 下的局部地图）对齐到地图坐标系，只重采样一次。
  const double resolution = costmap.metadata.resolution;
  const double co = std::cos(origin.yaw), so = std::sin(origin.yaw);
  const cv::Matx23d map_to_cells{
    co / resolution, so / resolution, -(co * origin.x + so * origin.y) / resolution - 0.5,
    -so / resolution, co / resolution, (so * origin.x - co * origin.y) / resolution - 0.5};
  auto sampling = compose(map_to_cells, env.pixel_to_map);
  for (int row = 0; row < 2; ++row) {
    sampling(row, 2) += sampling(row, 0) * left + sampling(row, 1) * top;
  }
  const double ratio = scale / resolution;
  const int radius = ratio > 1.0 ? static_cast<int>(std::ceil(ratio / std::sqrt(2.0))) : 0;
  double min_x = costmap.metadata.size_x, min_y = costmap.metadata.size_y;
  double max_x = -1, max_y = -1;
  for (const auto & point : std::array<cv::Point2d, 4>{{
      {0, 0}, {double(pixels - 1), 0},
      {0, double(pixels - 1)}, {double(pixels - 1), double(pixels - 1)}}})
  {
    const auto cell = apply(sampling, point.x, point.y);
    min_x = std::min(min_x, cell.x); min_y = std::min(min_y, cell.y);
    max_x = std::max(max_x, cell.x); max_y = std::max(max_y, cell.y);
  }
  const int x0 = std::clamp(int(std::floor(min_x)) - radius - 1, 0, int(costmap.metadata.size_x));
  const int y0 = std::clamp(int(std::floor(min_y)) - radius - 1, 0, int(costmap.metadata.size_y));
  const int x1 = std::clamp(int(std::ceil(max_x)) + radius + 2, 0, int(costmap.metadata.size_x));
  const int y1 = std::clamp(int(std::ceil(max_y)) + radius + 2, 0, int(costmap.metadata.size_y));
  cv::Mat view(pixels, pixels, CV_8UC3, cv::Scalar(unknown_color));
  if (x1 > x0 && y1 > y0) {
    cv::Mat raw(costmap.metadata.size_y, costmap.metadata.size_x, CV_8UC1,
      const_cast<std::uint8_t *>(costmap.data.data()));
    const auto crop = raw(cv::Rect(x0, y0, x1 - x0, y1 - y0));
    cv::Mat risk, blocked, unknown;
    cv::min(crop, 252, risk);
    cv::inRange(crop, 253, 254, blocked);
    cv::compare(crop, 255, unknown, cv::CMP_EQ);
    if (radius > 0) {
      // 缩小时分别保留障碍和未知掩膜，细墙不会被普通插值抹掉。
      const auto kernel = cv::getStructuringElement(cv::MORPH_RECT, {2 * radius + 1, 2 * radius + 1});
      cv::dilate(risk, risk, kernel);
      cv::dilate(blocked, blocked, kernel);
      cv::dilate(unknown, unknown, kernel, {-1, -1}, 1, cv::BORDER_CONSTANT, 255);
    }
    sampling(0, 2) -= x0; sampling(1, 2) -= y0;
    const int flags = cv::INTER_NEAREST | cv::WARP_INVERSE_MAP;
    cv::Mat costs, walls, missing;
    cv::warpAffine(risk, costs, sampling, view.size(), flags, cv::BORDER_CONSTANT, 0);
    cv::warpAffine(blocked, walls, sampling, view.size(), flags, cv::BORDER_CONSTANT, 0);
    cv::warpAffine(unknown, missing, sampling, view.size(), flags, cv::BORDER_CONSTANT, 255);
    for (int y = 0; y < pixels; ++y) {
      auto * output = view.ptr<cv::Vec3b>(y);
      const auto * values = costs.ptr<std::uint8_t>(y);
      const auto * obstacle = walls.ptr<std::uint8_t>(y);
      const auto * absent = missing.ptr<std::uint8_t>(y);
      for (int x = 0; x < pixels; ++x) {
        output[x] = obstacle[x] ? blocked_color : absent[x] ? unknown_color :
          values[x] >= 128 ? high_color : values[x] ? low_color : free_color;
      }
    }
  }

  // 逐段裁剪，而不是过滤框外点后重连；离开画面再进入时不会产生捷径。
  cv::Matx23d map_to_pixel;
  cv::invertAffineTransform(env.pixel_to_map, map_to_pixel);
  for (std::size_t i = 1; i < trail.size(); ++i) {
    if (trail[i].segment_start) {continue;}
    const auto a = apply(map_to_pixel, trail[i - 1].position.x, trail[i - 1].position.y);
    const auto b = apply(map_to_pixel, trail[i].position.x, trail[i].position.y);
    cv::Point start{cvRound(a.x - left), cvRound(a.y - top)};
    cv::Point end{cvRound(b.x - left), cvRound(b.y - top)};
    if (cv::clipLine(view.size(), start, end)) {
      cv::line(view, start, end, cv::Scalar(trail_color), 2, cv::LINE_AA);
    }
  }

  cv::Mat image(env.image_size, CV_8UC3, cv::Scalar(255, 255, 255));
  view.copyTo(image(env.map_rect));
  const cv::Scalar ink(55, 60, 70), blue(215, 100, 20);
  cv::putText(image, source + " surroundings  " + env.id, {left, 27},
    cv::FONT_HERSHEY_SIMPLEX, 0.65, ink, 1, cv::LINE_AA);
  cv::putText(image, "MAP  +X RIGHT  |  +Y UP  |  " +
    number(env.range_m) + " m square", {left, 50},
    cv::FONT_HERSHEY_SIMPLEX, 0.41, ink, 1, cv::LINE_AA);
  cv::rectangle(image, {left - 1, top - 1, pixels + 2, pixels + 2}, ink, 1);
  for (int i = 0; i <= 4; ++i) {
    const int delta = (pixels - 1) * i / 4;
    cv::putText(image, std::to_string(left + delta), {left + delta - 11, top + pixels + 20},
      cv::FONT_HERSHEY_SIMPLEX, 0.38, ink, 1, cv::LINE_AA);
    cv::putText(image, std::to_string(top + delta), {7, top + delta + 4},
      cv::FONT_HERSHEY_SIMPLEX, 0.38, ink, 1, cv::LINE_AA);
  }
  const cv::Point center{cvRound(cx), cvRound(cy)};
  const int arrow_length = std::min(56, pixels / 8);
  const cv::Point front{cvRound(cx + arrow_length * std::cos(env.robot.yaw)),
    cvRound(cy - arrow_length * std::sin(env.robot.yaw))};
  const cv::Scalar white(255, 255, 255);
  cv::arrowedLine(image, center, front, white, 8, cv::LINE_AA, 0, 0.32);
  cv::circle(image, center, 9, white, -1, cv::LINE_AA);
  cv::arrowedLine(image, center, front, blue, 4, cv::LINE_AA, 0, 0.32);
  cv::circle(image, center, 6, blue, -1, cv::LINE_AA);
  const double power = std::pow(10.0, std::floor(std::log10(env.range_m / 4)));
  const double scale_m = (env.range_m / 4 >= power * 5 ? 5 :
    env.range_m / 4 >= power * 2 ? 2 : 1) * power;
  const int length = cvRound(scale_m / scale), scale_y = top + pixels + 46;
  cv::line(image, {left, scale_y}, {left + length, scale_y}, ink, 2);
  cv::line(image, {left, scale_y - 4}, {left, scale_y + 4}, ink, 1);
  cv::line(image, {left + length, scale_y - 4}, {left + length, scale_y + 4}, ink, 1);
  cv::putText(image, number(scale_m) + " m | pixels: x right, y down",
    {left + length + 8, scale_y + 4}, cv::FONT_HERSHEY_SIMPLEX, 0.38, ink, 1, cv::LINE_AA);
  const std::array<cv::Vec3b, 6> colors{
    free_color, low_color, high_color, blocked_color, unknown_color, trail_color};
  const std::array<std::string, 6> names{"FREE", "LOW COST", "HIGH COST", "BLOCKED", "UNKNOWN", "TRAIL"};
  for (int i = 0; i < 6; ++i) {
    const int x = left + i * pixels / 6, y = top + pixels + 79;
    cv::rectangle(image, {x, y - 9, 9, 9}, cv::Scalar(colors[i]), -1);
    cv::rectangle(image, {x, y - 9, 9, 9}, ink, 1);
    cv::putText(image, names[i], {x + 13, y}, cv::FONT_HERSHEY_SIMPLEX, 0.34, ink, 1, cv::LINE_AA);
  }
  cv::putText(image, "Blue arrow: robot heading " +
    number(env.robot.yaw * 180 / CV_PI) + " deg", {left, top + pixels + 100},
    cv::FONT_HERSHEY_SIMPLEX, 0.4, blue, 1, cv::LINE_AA);
  return image;
}

}  // namespace vln_agent::navigation
