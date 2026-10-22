#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "vln_agent/tools/shared_resources.hpp"
#include "vln_agent/utils/json.hpp"

namespace rclcpp
{
class Node;
}

namespace vln_agent
{

/** @brief 插件的通用依赖，不包含模型会话、业务输入或事件接口。 */
struct ToolContext
{
  rclcpp::Node & node;                      ///< 非拥有引用，节点须晚于管理器销毁。
  std::shared_ptr<SharedResources> resources;  ///< 当前管理器的资源注册表。
  std::string group_id;                     ///< 配置中的工具组名称。
  Json group_settings;                     ///< 本组共享配置，管理器不解释业务字段。
  std::shared_ptr<const std::atomic_bool> stopping;  ///< 管理器关闭标志。
};

}  // namespace vln_agent

