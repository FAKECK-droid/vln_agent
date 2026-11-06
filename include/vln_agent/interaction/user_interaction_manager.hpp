#pragma once

#include <memory>
#include <string>
#include "vln_agent/interaction/user_interaction_sources.hpp"
#include "vln_agent/utils/json.hpp"

namespace rclcpp {class Node;}

namespace vln_agent
{
/** @brief 同步用户交互：每轮发送启用字段的完整快照，不保存完成历史。 */
class UserInteractionManager
{
public:
  UserInteractionManager(const std::shared_ptr<rclcpp::Node> & node,
                         const UserInteractionSources & sources);
  ~UserInteractionManager();
  void load(const std::string & global_yaml_path);
  /**
   * @brief 发送当前完整状态并合并插件提供的原生输入数组，不查询 pending，不消费 response/future。
   * @note 每批重复发送已启用的工具定义和状态；显示文本始终为整段 replace。
   * @param new_response 本次新增的完整模型响应。
   * @param tool_outputs 本次新增工具结果，保持原生模型输入结构。
   * @param finished_calls 与 tool_outputs 一一对应的完整终态元数据，含固定 wait_ms/run_ms。
   * @throws std::invalid_argument 工具结果缺少匹配的完整终态元数据。
   * @note 完成事件成功后不得重复投递，历史与去重由 Agent 管理。
   * @note 接口插件抛异常时保留当前批次，使用相同参数重试；已接收插件不重复收输出。
   * @note 所有调用及接口 ROS 回调须由外部串行安排，本类不启动线程。
   */
  Json exchange(const Json & new_response,
                const Json & tool_outputs,
                const Json & finished_calls);
  void shutdown();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace vln_agent
