#pragma once

#include <memory>

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;

/** @brief 按顺序启动多点导航，只等待目标接受或拒绝。 */
class NavigateThroughPosesStartNavigation : public ToolPluginBase
{
public:
  void initialize(const ToolContext & context, const Json & settings) override;
  ToolResult execute(const Json & arguments) override;
  /** @brief 无独立等待，共享导航资源由管理器统一关闭。 */
  void shutdown() noexcept override {}

private:
  std::shared_ptr<NavigationRuntime> navigation_;  ///< 本组共享资源，与单点导航、环顾互斥。
};

}  // namespace vln_agent
