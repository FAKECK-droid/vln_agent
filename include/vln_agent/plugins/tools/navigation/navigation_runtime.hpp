#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/drive_on_heading.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "vln_agent/tools/tool_plugin_base.hpp"
#include "vln_agent/plugins/tools/navigation/utils.hpp"

namespace tf2_ros {class Buffer; class TransformListener;}

namespace vln_agent
{

class ToolCallbacks;
class RgbdRuntime;

/** @brief 一组导航函数共用的 Action/Costmap 客户端、TF、任务状态和环境变换。 */
class NavigationRuntime : public SharedResource,
  public std::enable_shared_from_this<NavigationRuntime>
{
public:
  using Action = nav2_msgs::action::NavigateToPose;
  using ThroughPosesAction = nav2_msgs::action::NavigateThroughPoses;

  /** @brief 创建客户端，不等待服务端；所有函数复用同一外部节点。 */
  NavigationRuntime(rclcpp::Node & node, const Json & settings);
  ~NavigationRuntime() override;

  /** @brief poses 含一个位姿时启动单点导航，多个时启动有序多点导航。 */
  ToolResult start_navigation(const Json & arguments);
  /** @brief 沿车头轴平移后原地转向，等待动作完成。 */
  ToolResult move_relatively(const Json & arguments);
  /** @brief 从指定彩色图片的像素得到方位，等待转向完成。 */
  ToolResult lookat(RgbdRuntime & rgbd, const Json & arguments);
  /** @brief 完成八次采图和转向后直接返回环视图与 lookaround_id。 */
  ToolResult lookaround(RgbdRuntime & rgbd, const Json & settings);
  /** @brief 读取活动任务反馈；任务结束后由结果回调释放。 */
  ToolResult get_feedback(const Json & arguments);
  /** @brief 等待取消请求回复；只有最终 Action 结果才能将状态改为 canceled。 */
  ToolResult cancel_navigation(const Json & arguments);
  /**
   * @brief 获取代价地图并生成地图坐标系对齐的环境图。
   * @param arguments costmap 为 local/global，range_m 为观察正方形边长。
   * @return data.environment_id、公共 pose、images 元数据与原生 PNG。
   */
  ToolResult get_surroundings(const Json & arguments);
  /** @brief 读取当前 TF 位姿和本资源活动任务；不要求调用方持有 task_id。 */
  ToolResult get_robot_state();
  /**
   * @brief 使用快照还原像素位置，直接采用地图 yaw，复用 start_navigation。
   * @param arguments environment_id、pixel_x、pixel_y、yaw；yaw=0 朝图右方（地图 +X），正值逆时针。
   * @return 像素工具自身的结果，成功时含 task_id，供查询和取消。
   */
  ToolResult navigate_to_pixel_start_navigation(const Json & arguments);
  /** @brief 同一环境快照中的有序像素批次，转换后复用多点导航与自定义行为树。 */
  ToolResult navigate_through_pixels_start_navigation(const Json & arguments);
  /**
   * @brief 一次投影全部物体实例，生成停靠位姿后复用单点或多点导航。
   * @param rgbd 已装载的 RGB-D 共享资源。
   * @param arguments pair_id、use_segmentation、instances；实例可指定 stand_off_m，默认 0。
   * @return 投影明细、全局停靠位姿及导航任务；任一实例投影失败时整次不启动。
   */
  ToolResult navigate_to_objects(RgbdRuntime & rgbd, const Json & arguments);
  /** @brief 使用环视拼图实例投影，复用普通物体导航的停靠位姿与 Action 流程。 */
  ToolResult navigate_to_objects_surround(RgbdRuntime & rgbd, const Json & arguments);
  /** @brief 唤醒函数等待，尽力取消本客户端活动目标并停止自身回调。 */
  void shutdown() noexcept override;

private:
  using DriveAction = nav2_msgs::action::DriveOnHeading;
  using BackUpAction = nav2_msgs::action::BackUp;
  using SpinAction = nav2_msgs::action::Spin;
  using CancelCallback = rclcpp_action::Client<Action>::CancelCallback;
  using CancelGoal = std::function<void(CancelCallback)>;

  struct Task
  {
    std::string id;                         ///< 当前导航任务的内部标志。
    std::string action;                     ///< 当前 ROS Action 类型。
    CancelGoal cancel;                      ///< 目标接受后可用的取消函数。
    std::string status{"running"};          ///< 物理任务状态，不是函数调用状态。
    bool responded{false};                 ///< 是否已经收到目标接受或拒绝回复。
    Json feedback = Json::object();         ///< 最近一次反馈的业务字段。
    Json pose;                             ///< 最近一次反馈的小车位姿，含坐标系及可用采样时间。
  };

  /**
   * @brief 共用的 Action 发送、目标确认与取消所需任务管理。
   * @param goal 已组装的位姿和行为树路径。
   * @return 接受后的 task_id/status；环视转向可选择等待 Action 终态。
   */
  ToolResult sendGoal(const Action::Goal & goal, bool wait_for_result = false);
  ToolResult sendGoal(const ThroughPosesAction::Goal & goal);
  /** @brief 共用目标确认、反馈及终态释放逻辑。 */
  template<typename NavigationAction>
  ToolResult sendActionGoal(
    const typename NavigationAction::Goal & goal,
    const typename rclcpp_action::Client<NavigationAction>::SharedPtr & client,
    const char * action, bool wait_for_result = false);
  /** @brief 在平移和转向之间继续独占运动任务槽。 */
  ToolResult withRelativeMotion(const std::function<ToolResult()> & motion);
  /** @brief 通过 Nav2 行为沿底盘 x 轴平移。 */
  ToolResult driveRelative(double distance);
  /** @brief 通过 Nav2 行为按带符号弧度原地转向。 */
  ToolResult spinRelative(double yaw);
  /** @brief 单点与多点像素工具共用验证、快照转换及接受后消费逻辑。 */
  ToolResult startPixelNavigation(const Json & arguments, bool through_pixels);
  ToolResult navigateToObjects(RgbdRuntime & rgbd, const Json & arguments, bool surround);
  /** @brief 持锁生成任务状态和最新反馈结果。 */
  ToolResult snapshot(const std::shared_ptr<Task> & task);
  /** @brief 判断任务是否进入终态。 */
  static bool terminal(const std::string & status);
  /** @brief 定时读取最新 TF，按距离采样；缺失、陈旧或跳变时开始新线段。 */
  void sampleTrail();

  rclcpp::Node & node_;                     ///< 外部节点，生命周期由宿主管理。
  std::unique_ptr<ToolCallbacks> callbacks_;  ///< 独占回调组及其执行线程。
  rclcpp_action::Client<Action>::SharedPtr client_;  ///< 整组共用的单点导航客户端。
  rclcpp_action::Client<ThroughPosesAction>::SharedPtr through_poses_client_;  ///< 共用任务槽的多点客户端。
  rclcpp_action::Client<DriveAction>::SharedPtr drive_client_;
  rclcpp_action::Client<BackUpAction>::SharedPtr backup_client_;
  rclcpp_action::Client<SpinAction>::SharedPtr spin_client_;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr global_costmap_client_;  ///< 整组共用的全局代价地图客户端。
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr local_costmap_client_;   ///< 整组共用的局部代价地图客户端。

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;  ///< 有界 TF 缓存，供截图时的坐标转换。
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  ///< 复用本组专用回调线程。
  rclcpp::TimerBase::SharedPtr trail_timer_;  ///< 复用专用回调线程，不依赖导航反馈或截图。
  double trail_sample_interval_sec_{0.5};
  double trail_min_distance_m_{0.1};
  std::size_t trail_max_points_{2000};
  std::deque<navigation::TrailPoint> trail_;  ///< 受 mutex_ 保护，跨导航任务保留。
  cv::Point2d trail_last_position_;          ///< 上次有效采样位置，用于检测定位跳变。
  std::chrono::steady_clock::time_point trail_last_sample_;
  std::int64_t trail_last_stamp_ns_{0};
  bool trail_segment_start_{true};
  std::string map_frame_;   ///< 环境快照和导航目标使用的坐标系。
  std::string through_poses_behavior_tree_;  ///< 多点导航专用 XML，不能留空回退到 Nav2 默认树。
  std::string base_frame_;  ///< 机器人本体坐标系。
  double costmap_timeout_sec_{2.0};  ///< 服务发现和响应等待各自的时限。
  int image_size_{512};  ///< 地图区域边长，不包括标题与图例。
  std::size_t max_environments_{8};  ///< 未消费环境快照的数量上限。
  std::size_t max_pixel_goals_{8};  ///< 单次多点像素导航允许的最大目标数，必须为正整数。
  std::deque<navigation::Environment> environments_;  ///< 仅保存尚可引用的少量变换。
  std::uint64_t next_environment_id_{0};  ///< 递增 ID，仅计数而不保存历史。

  std::mutex mutex_;                       ///< 保护任务、关闭状态和环境元数据，不在图像处理期间持锁。
  std::condition_variable condition_;      ///< 目标、取消回复及关闭通知。
  std::shared_ptr<Task> task_;              ///< 唯一活动任务，收到终态后释放。
  std::uint64_t next_id_{0};                ///< 本次资源内递增的任务编号。
  std::size_t pending_goals_{0};            ///< 尚未收到回复的目标请求数。
  bool lookaround_running_{false};         ///< 在整轮采图和转向期间保留运动任务槽。
  bool relative_motion_running_{false};    ///< 短距离动作持有运动任务槽。
  bool closed_{false};                     ///< 关闭后唤醒等待并拒绝新操作。
  double shutdown_timeout_sec_;            ///< 关闭时等待未决目标回复的最长时间。
};

}  // namespace vln_agent
