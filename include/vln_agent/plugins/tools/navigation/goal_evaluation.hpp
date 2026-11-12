#pragma once

#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include "vln_agent/plugins/tools/navigation/utils.hpp"
#include "vln_agent/utils/json.hpp"

namespace vln_agent::navigation
{
/** @brief 目标评分参数，分数越低越优先。 */
struct GoalScoring
{
  double path_length_weight{1.0};  ///< 路径长度的代价权重。
  double clearance_weight{1.0};    ///< 障碍间距的收益权重，间距最多计 1 米。
  /** @brief 从函数 settings 读取权重。 */
  static GoalScoring fromSettings(const Json & settings);
};

/** @brief 单个接近位置的几何检查结果。 */
struct GoalCost
{
  std::string reason;           ///< 空表示 footprint 检查通过。
  double clearance_m{0};         ///< footprint 到障碍/地图边界的估计间距，最多 1 米。
};

/**
 * @brief 仅扫描目标附近的原始栅格，检查整个 footprint 并估计障碍间距。
 * @param map 本批唯一的全局代价地图，直接借用其 data。
 * @param origin 地图栅格原点在导航坐标系中的位姿。
 * @param footprint 车体坐标系中的真实 footprint，包含 Nav2 padding。
 * @param goal 导航坐标系中的候选位姿。
 * @return 越界、未知或碰撞原因，以及最多 1 米的间距。
 */
GoalCost checkGoal(const nav2_msgs::msg::Costmap & map, const Pose2D & origin,
  const std::vector<geometry_msgs::msg::Point> & footprint, const Pose2D & goal);
}  // namespace vln_agent::navigation
