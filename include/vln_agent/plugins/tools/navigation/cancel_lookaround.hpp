#pragma once

#include <memory>

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;

/** @brief 通过共用的 NavigateToPose 客户端取消环顾任务。 */
class CancelLookaround : public ToolPluginBase
{
public:
  /** @brief 取得 navigation 组共用的 Action 客户端、TF 与任务状态。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 使用启动时返回的 task_id 请求取消，不等待行为树结束。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 无独立资源；共享资源由 SharedResources 统一关闭。 */
  void shutdown() noexcept override {}

private:
  std::shared_ptr<NavigationRuntime> navigation_;  ///< navigation 组共享资源。
};
}  // namespace vln_agent
