#pragma once

#include <memory>
#include <string>
#include "vln_agent/interaction/user_interaction_sources.hpp"
#include "vln_agent/utils/json.hpp"

namespace rclcpp {class Node;}

namespace vln_agent
{
/** @brief 同步用户交互：查询一次、共享批次、全部插件接收后提交增量基线。 */
class UserInteractionManager
{
public:
  UserInteractionManager(const std::shared_ptr<rclcpp::Node> & node,
                         const UserInteractionSources & sources);
  ~UserInteractionManager();
  void load(const std::string & global_yaml_path);
  /**
   * @brief 直接合并插件提供的原生输入数组，不查询 pending，不消费 response/future。
   * @note latest_response 是完整模型响应；tool_outputs 是已就绪的函数结果数组。
   * @note 接口插件抛异常时保留当前批次，使用相同参数重试；已接收插件不重复收输出。
   * @note 所有调用及接口 ROS 回调须由外部串行安排，本类不启动线程。
   */
  Json exchange(const Json & latest_response = nullptr,
                const Json & tool_outputs = Json::array());
  void shutdown();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace vln_agent
