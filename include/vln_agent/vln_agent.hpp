#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "vln_agent/events/event_manager.hpp"
#include "vln_agent/interaction/user_interaction_manager.hpp"
#include "vln_agent/session/session_manager.hpp"
#include "vln_agent/tools/tool_manager.hpp"
#include "vln_agent/utils/json.hpp"

namespace vln_agent
{
/** @brief 单线程定频主循环；模型通信和工具执行由各自管理器异步完成。 */
class VLNAgent : public rclcpp::Node
{
public:
  explicit VLNAgent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VLNAgent() override;

  /** @brief make_shared 构造完成后调用一次，读取参数并装载全部组件。 */
  void init();
  /** @brief init 后运行，串行处理 ROS 回调和业务轮询，直到 ROS 退出。 */
  void run();
  /** @brief Agent 集中保留的输入/完整响应历史；只在主循环同线程查询。 */
  const Json & getHistory() const noexcept {return history_;}

private:
  void step();
  void addInputs(const Json & inputs);
  void shutdown();

  std::shared_ptr<EventManager> event_manager_;
  std::shared_ptr<UserInteractionManager> user_interaction_manager_;
  std::shared_ptr<ToolManager> tool_manager_;
  std::shared_ptr<SessionManager> session_manager_;
  std::vector<ToolCallHandle> active_calls_;
  Json history_ = Json::array();

  // 接口失败时原样续交这一批，成功后释放，避免重复执行工具或丢失展示事件。
  Json outgoing_response_;
  Json outgoing_tool_outputs_ = Json::array();
  bool exchange_pending_{false};
  double loop_hz_{20.0};
};
}  // namespace vln_agent
