#pragma once

#include <memory>

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;

/** @brief 以当前位姿启动环顾行为树，只等待 Action 接受或拒绝。 */
class StartLookaround : public ToolPluginBase
{
public:
  /** @brief 取得 navigation 组共用的 Action 客户端、TF 与任务状态。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 不接收模型参数，使用当前机器人位姿和配置的环顾 XML。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 无独立资源；共享资源由 SharedResources 统一关闭。 */
  void shutdown() noexcept override {}

private:
  std::shared_ptr<NavigationRuntime> navigation_;  ///< navigation 组共享资源。
};
}  // namespace vln_agent
