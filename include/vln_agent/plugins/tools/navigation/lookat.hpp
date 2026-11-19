#pragma once

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;

/** @brief 等待 Nav2 动作结束后交付结果的导航工具。 */
class Lookat : public ToolPluginBase
{
public:
  /** @brief 复用导航组的共享资源。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 等待动作结果。 */
  ToolResult execute(const Json & arguments) override;
  void shutdown() noexcept override {}

private:
  std::shared_ptr<SharedResources> resources_;
  std::string rgbd_resource_;
  std::shared_ptr<NavigationRuntime> navigation_;
};
}  // namespace vln_agent
