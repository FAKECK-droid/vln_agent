#pragma once

#include <memory>
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;

/** @brief 查询当前导航任务的反馈，自动识别单点或多点 Action。 */
class GetNavigationFeedback : public ToolPluginBase
{
public:
  void initialize(const ToolContext & context, const Json & settings) override;
  ToolResult execute(const Json & arguments) override;
  void shutdown() noexcept override {}

private:
  std::shared_ptr<NavigationRuntime> navigation_;
};
}  // namespace vln_agent
