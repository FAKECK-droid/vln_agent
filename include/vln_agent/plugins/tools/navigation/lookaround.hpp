#pragma once

#include <memory>
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;
class Lookaround : public ToolPluginBase
{
public:
  void initialize(const ToolContext & context, const Json & settings) override;
  ToolResult execute(const Json & arguments) override;
  void shutdown() noexcept override {}
private:
  std::shared_ptr<SharedResources> resources_;
  std::string rgbd_resource_;
  std::shared_ptr<NavigationRuntime> navigation_;
  Json settings_;
};
}  // namespace vln_agent
