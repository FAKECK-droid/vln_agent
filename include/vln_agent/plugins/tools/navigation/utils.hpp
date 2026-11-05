#pragma once

#include <string>
#include <nav2_msgs/msg/costmap.hpp>
#include <opencv2/core.hpp>

namespace vln_agent::navigation
{

/** @brief 平面位姿；距离使用米，朝向为所在坐标系中的逆时针弧度。 */
struct Pose2D
{
  double x{0};    ///< 平面 x 坐标。
  double y{0};    ///< 平面 y 坐标。
  double yaw{0};  ///< 绕 z 轴的朝向。
};

/** @brief 一张环境图的轻量几何快照；不持有地图数据或编码后的图片。 */
struct Environment
{
  std::string id;                 ///< 当前 Runtime 内唯一的环境 ID。
  std::string frame_id;           ///< 还原后的导航目标坐标系，通常为 map。
  Pose2D robot;                   ///< 截图时机器人在 frame_id 中的位姿。
  double range_m{0};              ///< 正方形观察区域边长，单位米。
  double meters_per_pixel{0};     ///< 地图部分每像素对应的米数。
  cv::Rect map_rect;              ///< 完整 PNG 中的地图矩形；不包含标题和图例。
  cv::Size image_size;            ///< 包含标题、标尺和图例的完整 PNG 尺寸。
  cv::Matx23d pixel_to_map;        ///< 完整图片像素到 frame_id 平面坐标的仿射变换。
};

/**
 * @brief 裁剪机器人周围正方形，合并旋转、缩放与坐标翻转，并绘制说明。
 * @param costmap 服务返回的原始 0～255 代价地图，只读借用其 data。
 * @param origin_in_map 地图栅格原点在导航坐标系中的平面位姿。
 * @param environment 输入机器人位姿、范围和 ID；输出像素几何信息。
 * @param map_pixels 地图部分的像素边长，不包括图片边距。
 * @param source 标题中的 local/global 来源。
 * @return 机器人朝上的 BGR 图片；缩小时保守保留障碍和未知区域。
 */
cv::Mat renderSurroundings(
  const nav2_msgs::msg::Costmap & costmap, const Pose2D & origin_in_map,
  Environment & environment, int map_pixels, const std::string & source);

/**
 * @brief 将对应快照完整 PNG 中的像素还原为导航坐标。
 * @param environment 生成该图片时保存的几何快照。
 * @param x 完整图片横坐标，从左向右增加。
 * @param y 完整图片纵坐标，从上向下增加。
 * @return environment.frame_id 中的平面位置。
 * @throws std::invalid_argument 像素落在地图矩形之外或不是有限值。
 */
cv::Point2d pixelToMap(const Environment & environment, double x, double y);

}  // namespace vln_agent::navigation
