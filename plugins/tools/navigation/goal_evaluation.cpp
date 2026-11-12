#include "vln_agent/plugins/tools/navigation/goal_evaluation.hpp"
#include "vln_agent/plugins/tools/navigation/navigation_runtime.hpp"
#include "vln_agent/plugins/tools/rgbd/rgbd_runtime.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <optional>
#include <opencv2/imgproc.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace vln_agent::navigation
{
GoalScoring GoalScoring::fromSettings(const Json & settings)
{
  return {settings.value("path_length_weight", 1.0), settings.value("clearance_weight", 1.0)};
}

GoalCost checkGoal(const nav2_msgs::msg::Costmap & map, const Pose2D & origin,
  const std::vector<geometry_msgs::msg::Point> & footprint, const Pose2D & goal)
{
  const int width = map.metadata.size_x, height = map.metadata.size_y;
  const double resolution = map.metadata.resolution;
  const double co = std::cos(origin.yaw), so = std::sin(origin.yaw);
  const auto cell = [&](double x, double y) {
      const double dx = x - origin.x, dy = y - origin.y;
      return cv::Point2f((co * dx + so * dy) / resolution,
        (-so * dx + co * dy) / resolution);
    };
  const auto center = cell(goal.x, goal.y);
  if (center.x < 0 || center.y < 0 || center.x >= width || center.y >= height) {
    return {"GOAL_OUTSIDE_MAP", 0};
  }
  const auto center_cost = map.data[static_cast<int>(center.y) * width + static_cast<int>(center.x)];
  if (center_cost >= 253) {
    return {center_cost == 255 ? "UNKNOWN_SPACE" : "GOAL_OCCUPIED", 0};
  }

  std::vector<cv::Point2f> polygon;
  std::vector<cv::Point> raster;
  polygon.reserve(footprint.size()); raster.reserve(footprint.size());
  const double c = std::cos(goal.yaw), s = std::sin(goal.yaw);
  for (const auto & point : footprint) {
    const auto p = cell(goal.x + c * point.x - s * point.y,
      goal.y + s * point.x + c * point.y);
    if (p.x < 0 || p.y < 0 || p.x >= width || p.y >= height) {
      return {"GOAL_OUTSIDE_MAP", 0};
    }
    polygon.push_back(p);
    raster.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));
  }
  const auto bounds = cv::boundingRect(raster);
  cv::Mat mask = cv::Mat::zeros(bounds.height, bounds.width, CV_8U);
  for (auto & point : raster) {point -= bounds.tl();}
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{raster}, cv::Scalar(255));
  for (int y = 0; y < mask.rows; ++y) {
    const auto * row = mask.ptr<unsigned char>(y);
    const auto * costs = map.data.data() + (y + bounds.y) * width + bounds.x;
    for (int x = 0; x < mask.cols; ++x) {
      if (row[x] && costs[x] >= 254) {
        return {costs[x] == 255 ? "UNKNOWN_SPACE" : "FOOTPRINT_COLLISION", 0};
      }
    }
  }

  // 间距只需用于排序；扫描 footprint 周围最多 1 米，避免整张地图距离变换。
  double clearance = std::min({1.0, bounds.x * resolution, bounds.y * resolution,
      (width - bounds.x - bounds.width) * resolution,
      (height - bounds.y - bounds.height) * resolution});
  const int radius = static_cast<int>(std::ceil(1.0 / resolution)) + 1;
  const int x0 = std::max(0, bounds.x - radius), y0 = std::max(0, bounds.y - radius);
  const int x1 = std::min(width, bounds.x + bounds.width + radius);
  const int y1 = std::min(height, bounds.y + bounds.height + radius);
  for (int y = y0; y < y1; ++y) {
    const auto * costs = map.data.data() + y * width;
    for (int x = x0; x < x1; ++x) {
      if (costs[x] < 254) {continue;}
      const double distance = -cv::pointPolygonTest(polygon, cv::Point2f(x + 0.5, y + 0.5), true);
      clearance = std::min(clearance, std::max(0.0, (distance - std::sqrt(0.5)) * resolution));
    }
  }
  return {"", clearance};
}
}  // namespace vln_agent::navigation

namespace vln_agent
{
namespace
{
/** @brief 纯规划目标可能已结束或正在关闭，取消失败由本次超时结果表达。 */
void cancelPlan(
  const std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>> & client,
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr & goal) noexcept
{
  try {client->async_cancel_goal(goal);}
  catch (const std::exception &) {}
}

/** @brief 仅在本次调用中保存候选，不保留评估或规划历史。 */
struct Evaluation
{
  int pixel_x{}, pixel_y{};                 ///< 模型使用的返回图像像素。
  double stand_off_m{};                    ///< 目标到小车中心的水平距离。
  Json target;                             ///< pair 投影得到的固定坐标位置。
  std::optional<navigation::Pose2D> goal;    ///< 生成的接近位姿。
  bool evaluated{false}, reachable{false};  ///< 未完成规划时 reachable 输出 null。
  std::string reason;                      ///< 深度、地图或规划原因。
  double clearance{}, length{}, score{};    ///< 可达目标的评分数据。
};

Json goalJson(const navigation::Pose2D & pose, const std::string & frame)
{
  return {{"frame_id", frame}, {"x", pose.x}, {"y", pose.y}, {"yaw", pose.yaw}};
}
}  // namespace

ToolResult NavigationRuntime::evaluateImageGoals(RgbdRuntime & rgbd, const Json & arguments,
  const navigation::GoalScoring & scoring, std::size_t approaches)
{
  using Clock = std::chrono::steady_clock;
  std::lock_guard<std::mutex> serial(evaluation_mutex_);
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {return ToolResult::failure("TOOL_MANAGER_CLOSED", "导航资源已经关闭");}
    if (task_) {return ToolResult::failure("NAVIGATION_BUSY", "活动导航期间不提交额外规划请求");}
    footprint = footprint_;
  }
  const auto & inputs = arguments.at("goals");
  if (inputs.empty() || inputs.size() > 8) {
    return ToolResult::failure("INVALID_ARGUMENTS", "每次评估 1 到 8 个图像目标");
  }
  std::vector<cv::Point> pixels;
  for (const auto & input : inputs) {
    pixels.emplace_back(input.at("pixel_x").get<int>(), input.at("pixel_y").get<int>());
  }
  const auto id = arguments.at("image_pair_id").get<std::string>();
  const auto projections = rgbd.getDepthPoints(id, pixels, map_frame_);
  std::vector<Evaluation> evaluations;
  evaluations.reserve(inputs.size() * approaches);
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    Evaluation item;
    item.pixel_x = pixels[i].x; item.pixel_y = pixels[i].y;
    item.stand_off_m = inputs[i].at("stand_off_m").get<double>();
    if (projections[i].payload.at("ok").get<bool>()) {
      const auto & data = projections[i].payload.at("data");
      item.target = {{"frame_id", map_frame_}, {"x", data.at("x")},
        {"y", data.at("y")}, {"z", data.at("z")}};
    } else {
      item.evaluated = true;
      item.reason = projections[i].payload.at("code").get<std::string>();
    }
    evaluations.push_back(std::move(item));
  }

  const auto report = [&]() {
      Json rows = Json::array(), best = nullptr;
      double best_score = 0;
      for (std::size_t i = 0; i < evaluations.size(); ++i) {
        const auto & item = evaluations[i];
        Json row{{"index", i}, {"pixel_x", item.pixel_x}, {"pixel_y", item.pixel_y},
          {"stand_off_m", item.stand_off_m},
          {"reachable", item.evaluated ? Json(item.reachable) : Json(nullptr)}};
        if (!item.target.is_null()) {row["target"] = item.target;}
        if (item.goal) {
          row["goal"] = goalJson(*item.goal, map_frame_);
          row["goal_clearance_m"] = item.clearance;
        }
        if (!item.reason.empty()) {row["reason"] = item.reason;}
        if (item.reachable) {
          row["path_length_m"] = item.length;
          row["score"] = item.score;
          if (best.is_null() || item.score < best_score) {best = row; best_score = item.score;}
        }
        rows.push_back(std::move(row));
      }
      return ToolResult::success({{"image_pair_id", id}, {"best_goal", std::move(best)},
          {"goals", std::move(rows)}, {"clearance_cap_m", 1.0},
          {"selection_weights", {{"path_length_weight", scoring.path_length_weight},
              {"clearance_weight", scoring.clearance_weight}}}});
    };
  const auto fail = [&](const std::string & code, const std::string & message) {
      auto result = report();
      result.payload["ok"] = false;
      result.payload["code"] = code; result.payload["message"] = message;
      return result;
    };
  if (std::all_of(evaluations.begin(), evaluations.end(),
      [](const auto & item) {return item.evaluated;})) {return report();}
  if (!footprint || footprint->polygon.points.size() < 3) {
    return fail("FOOTPRINT_UNAVAILABLE", "尚未收到 Nav2 的有效 footprint");
  }

  const auto timeout = std::chrono::duration<double>(costmap_timeout_sec_);
  if (!global_costmap_client_->wait_for_service(timeout)) {
    return fail("COSTMAP_UNAVAILABLE", "全局代价地图服务不可用");
  }
  auto future = global_costmap_client_->async_send_request(
    std::make_shared<nav2_msgs::srv::GetCostmap::Request>());
  if (future.wait_for(timeout) != std::future_status::ready) {
    global_costmap_client_->remove_pending_request(future);
    return fail("COSTMAP_TIMEOUT", "等待全局代价地图超时");
  }
  const auto response = future.get();
  const auto & map = response->map;
  if (map.metadata.size_x == 0 || map.metadata.size_y == 0) {
    return fail("EMPTY_COSTMAP", "全局代价地图尚无数据");
  }
  geometry_msgs::msg::TransformStamped robot, grid_transform, footprint_transform;
  try {
    robot = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    grid_transform = tf_buffer_->lookupTransform(map_frame_, map.header.frame_id, tf2::TimePointZero);
    footprint_transform = tf_buffer_->lookupTransform(base_frame_, footprint->header.frame_id,
      rclcpp::Time(footprint->header.stamp), rclcpp::Duration::from_seconds(costmap_timeout_sec_));
  } catch (const tf2::TransformException & error) {
    return fail("TF_UNAVAILABLE", error.what());
  }
  std::vector<geometry_msgs::msg::Point> shape;
  for (const auto & input : footprint->polygon.points) {
    geometry_msgs::msg::PointStamped point;
    point.point.x = input.x; point.point.y = input.y; point.point.z = input.z;
    tf2::doTransform(point, point, footprint_transform);
    shape.push_back(point.point);
  }
  const double grid_yaw = tf2::getYaw(grid_transform.transform.rotation);
  const auto & origin_pose = map.metadata.origin;
  const navigation::Pose2D origin{
    grid_transform.transform.translation.x + std::cos(grid_yaw) * origin_pose.position.x -
    std::sin(grid_yaw) * origin_pose.position.y,
    grid_transform.transform.translation.y + std::sin(grid_yaw) * origin_pose.position.x +
    std::cos(grid_yaw) * origin_pose.position.y,
    grid_yaw + tf2::getYaw(origin_pose.orientation)};

  std::vector<Evaluation> expanded;
  for (const auto & input : evaluations) {
    if (input.evaluated) {expanded.push_back(input); continue;}
    const double tx = input.target.at("x"), ty = input.target.at("y");
    const double direction = std::atan2(robot.transform.translation.y - ty,
      robot.transform.translation.x - tx);
    const std::size_t count = input.stand_off_m > 0 ? approaches : 1;
    for (std::size_t j = 0; j < count; ++j) {
      auto item = input;
      const double angle = direction + 2 * M_PI * j / count;
      item.goal = navigation::Pose2D{
        tx + item.stand_off_m * std::cos(angle),
        ty + item.stand_off_m * std::sin(angle), std::remainder(angle + M_PI, 2 * M_PI)};
      const auto cost = navigation::checkGoal(map, origin, shape, *item.goal);
      item.clearance = cost.clearance_m;
      if (!cost.reason.empty()) {item.evaluated = true; item.reason = cost.reason;}
      expanded.push_back(std::move(item));
    }
  }
  evaluations = std::move(expanded);
  if (std::all_of(evaluations.begin(), evaluations.end(),
      [](const auto & item) {return item.evaluated;})) {return report();}
  if (!planner_->wait_for_action_server(std::chrono::duration<double>(planning_timeout_sec_))) {
    return fail("PLANNER_UNAVAILABLE", "规划 Action 不可用");
  }

  // Nav2 Humble 规划 Action 有当前/待处理目标；批内串行，避免候选相互抢占。
  for (auto & item : evaluations) {
    if (item.evaluated) {continue;}
    PlanAction::Goal goal;
    goal.use_start = true;
    goal.start.header.frame_id = goal.goal.header.frame_id = map_frame_;
    goal.start.header.stamp = goal.goal.header.stamp = node_.now();
    goal.start.pose.position.x = robot.transform.translation.x;
    goal.start.pose.position.y = robot.transform.translation.y;
    goal.start.pose.orientation = robot.transform.rotation;
    goal.goal.pose.position.x = item.goal->x; goal.goal.pose.position.y = item.goal->y;
    goal.goal.pose.orientation.z = std::sin(item.goal->yaw / 2);
    goal.goal.pose.orientation.w = std::cos(item.goal->yaw / 2);
    const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(planning_timeout_sec_));
    const auto client = planner_;
    auto expired = std::make_shared<std::atomic_bool>(false);
    rclcpp_action::Client<PlanAction>::SendGoalOptions options;
    options.goal_response_callback =
      [expired, weak = std::weak_ptr<rclcpp_action::Client<PlanAction>>(client)](auto handle) {
        if (expired->load() && handle) {
          if (const auto owner = weak.lock()) {cancelPlan(owner, handle);}
        }
      };
    auto sent = client->async_send_goal(goal, options);
    if (sent.wait_until(deadline) != std::future_status::ready) {
      expired->store(true);
      if (sent.wait_for(std::chrono::seconds(0)) == std::future_status::ready && sent.get()) {
        cancelPlan(client, sent.get());
      }
      resetPlanner();
      return fail("PLANNING_TIMEOUT", "等待规划目标确认超时，未完成的候选 reachable 为 null");
    }
    const auto handle = sent.get();
    item.evaluated = true;
    if (!handle) {item.reason = "PLANNING_REJECTED"; continue;}
    auto result = client->async_get_result(handle);
    if (result.wait_until(deadline) != std::future_status::ready) {
      item.evaluated = false;
      cancelPlan(client, handle);
      resetPlanner();
      return fail("PLANNING_TIMEOUT", "等待路径规划超时，未完成的候选 reachable 为 null");
    }
    const auto planned = result.get();
    if (planned.code != rclcpp_action::ResultCode::SUCCEEDED || planned.result->path.poses.empty()) {
      item.reason = "NO_PATH"; continue;
    }
    const auto & poses = planned.result->path.poses;
    for (std::size_t i = 1; i < poses.size(); ++i) {
      const auto & a = poses[i - 1].pose.position;
      const auto & b = poses[i].pose.position;
      item.length += std::hypot(b.x - a.x, b.y - a.y);
    }
    item.reachable = true;
    item.score = scoring.path_length_weight * item.length -
      scoring.clearance_weight * item.clearance;
  }
  return report();
}
}  // namespace vln_agent
