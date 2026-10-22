#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{

class ToolCallbacks;

/** @brief 一组导航函数共用的 Action 客户端、任务记录及反馈状态。 */
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
  /** @brief 读取指定任务的当前反馈；已完成任务记录保留到资源销毁。 */
  ToolResult get_feedback(const Json & arguments);
  /** @brief 等待取消请求回复；只有最终 Action 结果才能将状态改为 canceled。 */
  ToolResult cancel_navigation(const Json & arguments);
  /** @brief 唤醒函数等待，尽力取消本客户端活动目标并停止自身回调。 */
  void shutdown() noexcept override;

private:
  struct Task
  {
    GoalHandle::SharedPtr goal;             ///< 接受目标后得到的句柄。
    std::string status{"running"};          ///< 物理任务状态，不是函数调用状态。
    bool responded{false};                 ///< 是否已经收到目标接受或拒绝回复。
    Json feedback = Json::object();         ///< 最近一次反馈的业务字段。
  };

  /** @brief 在持有状态锁时构造一个已有任务的成功查询结果。 */
  ToolResult snapshot(const std::string & id) const;
  /** @brief 判断任务是否进入终态。 */
  static bool terminal(const std::string & status);

  rclcpp::Node & node_;                     ///< 外部节点，生命周期由宿主管理。
  std::unique_ptr<ToolCallbacks> callbacks_;  ///< 独占回调组及其执行线程。
  rclcpp_action::Client<Action>::SharedPtr client_;  ///< 整组共用的导航客户端。
  std::mutex mutex_;                       ///< 只保护任务表和关闭状态。
  std::condition_variable condition_;      ///< 目标、取消回复及关闭通知。
  std::unordered_map<std::string, Task> tasks_;  ///< 包含历史终态的任务表。
  std::string active_;                     ///< 唯一活动任务，空字符串表示空闲。
  std::uint64_t next_id_{0};                ///< 本次资源内递增的任务编号。
  std::size_t pending_goals_{0};            ///< 尚未收到回复的目标请求数。
  bool closed_{false};                     ///< 关闭后唤醒等待并拒绝新操作。
  double shutdown_timeout_sec_;            ///< 关闭时等待未决目标回复的最长时间。
};

}  // namespace vln_agent
