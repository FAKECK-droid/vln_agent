#pragma once

#include <memory>
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;

/** @brief 根据 poses 数量自动启动单点或多点导航，只等待目标接受。 */
class StartNavigation : public ToolPluginBase
{
public:
  void initialize(const ToolContext & context, const Json & settings) override;
  ToolResult execute(const Json & arguments) override;
  void shutdown() noexcept override {}

private:
  std::shared_ptr<NavigationRuntime> navigation_;
};
}  // namespace vln_agent
