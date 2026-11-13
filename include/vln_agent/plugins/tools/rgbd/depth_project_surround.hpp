#pragma once

#include <memory>
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class RgbdRuntime;
/** @brief 还原环视拼图提示并复用同组 RGB-D 深度投影。 */
class DepthProjectSurround : public ToolPluginBase
{
public:
  void initialize(const ToolContext & context, const Json & settings) override;
  ToolResult execute(const Json & arguments) override;
  void shutdown() noexcept override {}
private:
  std::shared_ptr<RgbdRuntime> rgbd_;
};
}  // namespace vln_agent
