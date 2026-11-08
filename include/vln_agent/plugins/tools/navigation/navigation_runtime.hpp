#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "vln_agent/tools/tool_plugin_base.hpp"
#include "vln_agent/plugins/tools/navigation/utils.hpp"

namespace tf2_ros {class Buffer; class TransformListener;}

namespace vln_agent
{

class ToolCallbacks;

/** @brief 一组导航函数共用的 Action/Costmap 客户端、TF、任务状态和环境变换。 */
class NavigationRuntime : public SharedResource,
  public std::enable_shared_from_this<NavigationRuntime>
{
public:
  using Action = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  /** @brief 创建客户端，不等待服务端；所有函数复用同一外部节点。 */
  NavigationRuntime(rclcpp::Node & node, const Json & settings);
  ~NavigationRuntime() override;

  /** @brief 等待目标接受或拒绝；接受后返回 task_id，不等待物理导航完成。 */
  ToolResult start_navigation(const Json & arguments);
  /** @brief 以当前位姿启动环顾行为树；接受后返回 task_id，终态由行为树事件上报。 */
  ToolResult start_lookaround();
  /** @brief 读取当前任务反馈；终态首次返回后即释放，由 Agent 保存历史。 */
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
  /** @brief 唤醒函数等待，尽力取消本客户端活动目标并停止自身回调。 */
  void shutdown() noexcept override;

private:
  struct Task
  {
    std::string id;                         ///< 当前导航任务的内部标志。
    GoalHandle::SharedPtr goal;             ///< 接受目标后得到的句柄。
    std::string status{"running"};          ///< 物理任务状态，不是函数调用状态。
    bool release_on_finish{false};          ///< 环顾由事件交付结果，Action 结束后直接释放任务。
    bool responded{false};                 ///< 是否已经收到目标接受或拒绝回复。
    Json feedback = Json::object();         ///< 最近一次反馈的业务字段。
    Json pose;                             ///< 最近一次反馈的小车位姿，含坐标系及可用采样时间。
  };

  /**
   * @brief 共用的 Action 发送、目标确认与取消所需任务管理。
   * @param goal 已组装的位姿和行为树路径。
   * @param release_on_finish 为 true 时不收集反馈，终态回调直接释放内部任务。
   * @return 接受后的 task_id/status 或启动失败信息，不等待动作完成。
   */
  ToolResult sendGoal(const Action::Goal & goal, bool release_on_finish);
  /** @brief 持锁生成当前任务结果，并在交付终态时释放资源内部引用。 */
  ToolResult snapshot(const std::shared_ptr<Task> & task);
  /** @brief 判断任务是否进入终态。 */
  static bool terminal(const std::string & status);

  rclcpp::Node & node_;                     ///< 外部节点，生命周期由宿主管理。
  std::unique_ptr<ToolCallbacks> callbacks_;  ///< 独占回调组及其执行线程。
  rclcpp_action::Client<Action>::SharedPtr client_;  ///< 整组共用的导航客户端。
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr global_costmap_client_;  ///< 整组共用的全局代价地图客户端。
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr local_costmap_client_;   ///< 整组共用的局部代价地图客户端。

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;  ///< 有界 TF 缓存，供截图时的坐标转换。
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  ///< 复用本组专用回调线程。
  std::string map_frame_;   ///< 环境快照和导航目标使用的坐标系。
  std::string lookaround_behavior_tree_;  ///< 环顾 XML 路径；为空时从 vln_behavior_tree 安装目录定位。
  std::string base_frame_;  ///< 机器人本体坐标系。
  double costmap_timeout_sec_{2.0};  ///< 服务发现和响应等待各自的时限。
  int image_size_{512};  ///< 地图区域边长，不包括标题与图例。
  std::size_t max_environments_{8};  ///< 未消费环境快照的数量上限。
  std::deque<navigation::Environment> environments_;  ///< 仅保存尚可引用的少量变换。
  std::uint64_t next_environment_id_{0};  ///< 递增 ID，仅计数而不保存历史。

  std::mutex mutex_;                       ///< 保护任务、关闭状态和环境元数据，不在图像处理期间持锁。
  std::condition_variable condition_;      ///< 目标、取消回复及关闭通知。
  std::shared_ptr<Task> task_;              ///< 唯一活动任务或尚待消费的终态。
  std::uint64_t next_id_{0};                ///< 本次资源内递增的任务编号。
  std::size_t pending_goals_{0};            ///< 尚未收到回复的目标请求数。
  bool closed_{false};                     ///< 关闭后唤醒等待并拒绝新操作。
  double shutdown_timeout_sec_;            ///< 关闭时等待未决目标回复的最长时间。
};

}  // namespace vln_agent
