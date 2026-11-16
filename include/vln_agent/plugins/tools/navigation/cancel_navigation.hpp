#pragma once

#include <memory>
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;

/** @brief 按 task_id 自动选择 Action 并请求取消导航。 */
class CancelNavigation : public ToolPluginBase
{
public:
  void initialize(const ToolContext & context, const Json & settings) override;
  ToolResult execute(const Json & arguments) override;
  void shutdown() noexcept override {}

private:
  std::shared_ptr<NavigationRuntime> navigation_;
};
}  // namespace vln_agent
