#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <nav2_msgs/action/navigate_to_pose.hpp>
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
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using ThroughPosesAction = nav2_msgs::action::NavigateThroughPoses;
  using ThroughPosesGoalHandle = rclcpp_action::ClientGoalHandle<ThroughPosesAction>;

  /** @brief 创建客户端，不等待服务端；所有函数复用同一外部节点。 */
  NavigationRuntime(rclcpp::Node & node, const Json & settings);
  ~NavigationRuntime() override;

  /** @brief 等待目标接受或拒绝；接受后返回 task_id，不等待物理导航完成。 */
  ToolResult start_navigation(const Json & arguments);
  /** @brief 按输入顺序提交多点目标，始终指定自定义多点行为树。 */
  ToolResult start_through_poses_navigation(const Json & arguments);
  /** @brief 查询活动多点任务，包含剩余位姿数量。 */
  ToolResult get_through_poses_feedback(const Json & arguments);
  /** @brief 请求取消多点任务，最终 Action 结果决定是否已取消。 */
  ToolResult cancel_through_poses_navigation(const Json & arguments);
  /** @brief 以当前位姿启动环顾行为树；接受后返回 task_id，终态由行为树事件上报。 */
  ToolResult start_lookaround();
  /** @brief 读取活动任务反馈；任务结束后由结果回调释放。 */
  ToolResult get_feedback(const Json & arguments);
  /** @brief 等待取消请求回复；只有最终 Action 结果才能将状态改为 canceled。 */
  ToolResult cancel_navigation(const Json & arguments);
  /**
   * @brief 获取代价地图并生成机器人朝上的环境图。
   * @param arguments costmap 为 local/global，range_m 为观察正方形边长。
   * @return data.environment_id、公共 pose、images 元数据与原生 PNG。
   */
  ToolResult get_surroundings(const Json & arguments);
  /**
   * @brief 使用快照还原像素位置与相对图片 yaw，复用 start_navigation。
   * @param arguments environment_id、pixel_x、pixel_y、yaw；yaw=0 朝图上方，正值逆时针。
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
  /** @brief 唤醒函数等待，尽力取消本客户端活动目标并停止自身回调。 */
  void shutdown() noexcept override;

private:
  struct Task
  {
    std::string id;                         ///< 当前导航任务的内部标志。
    GoalHandle::SharedPtr goal;             ///< 单点目标句柄，与多点句柄二选一。
    ThroughPosesGoalHandle::SharedPtr through_poses_goal;  ///< 多点目标句柄。
    bool through_poses{false};              ///< 区分 Action，避免反馈和取消工具混用。
    std::string status{"running"};          ///< 物理任务状态，不是函数调用状态。
    bool responded{false};                 ///< 是否已经收到目标接受或拒绝回复。
    Json feedback = Json::object();         ///< 最近一次反馈的业务字段。
    Json pose;                             ///< 最近一次反馈的小车位姿，含坐标系及可用采样时间。
  };

  /**
   * @brief 共用的 Action 发送、目标确认与取消所需任务管理。
   * @param goal 已组装的位姿和行为树路径。
   * @return 接受后的 task_id/status 或启动失败信息，不等待动作完成。
   */
  ToolResult sendGoal(const Action::Goal & goal);
  ToolResult sendGoal(const ThroughPosesAction::Goal & goal);
  /** @brief 两类 Action 共用目标确认、反馈及终态释放逻辑。 */
  template<typename NavigationAction>
  ToolResult sendActionGoal(
    const typename NavigationAction::Goal & goal,
    const typename rclcpp_action::Client<NavigationAction>::SharedPtr & client);
  /** @brief 单点与多点像素工具共用验证、快照转换及接受后消费逻辑。 */
  ToolResult startPixelNavigation(const Json & arguments, bool through_pixels);
  ToolResult getFeedback(const Json & arguments, bool through_poses);
  ToolResult cancelNavigation(const Json & arguments, bool through_poses);
  /** @brief 持锁生成任务状态和最新反馈结果。 */
  ToolResult snapshot(const std::shared_ptr<Task> & task);
  /** @brief 判断任务是否进入终态。 */
  static bool terminal(const std::string & status);

  rclcpp::Node & node_;                     ///< 外部节点，生命周期由宿主管理。
  std::unique_ptr<ToolCallbacks> callbacks_;  ///< 独占回调组及其执行线程。
  rclcpp_action::Client<Action>::SharedPtr client_;  ///< 整组共用的单点导航客户端。
  rclcpp_action::Client<ThroughPosesAction>::SharedPtr through_poses_client_;  ///< 共用任务槽的多点客户端。
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr global_costmap_client_;  ///< 整组共用的全局代价地图客户端。
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr local_costmap_client_;   ///< 整组共用的局部代价地图客户端。

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;  ///< 有界 TF 缓存，供截图时的坐标转换。
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  ///< 复用本组专用回调线程。
  std::string map_frame_;   ///< 环境快照和导航目标使用的坐标系。
  std::string lookaround_behavior_tree_;  ///< 环顾 XML 路径；为空时从 vln_behavior_tree 安装目录定位。
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
  bool closed_{false};                     ///< 关闭后唤醒等待并拒绝新操作。
  double shutdown_timeout_sec_;            ///< 关闭时等待未决目标回复的最长时间。
};

}  // namespace vln_agent
