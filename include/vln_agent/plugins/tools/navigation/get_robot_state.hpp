#pragma once

#include <memory>
#include <string>
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;

/** @brief 汇总机器人位姿、活动任务和可选的相机时效信息。 */
class GetRobotState : public ToolPluginBase
{
public:
  /** @brief 复用导航资源；rgbd_resource 为空时不查询相机状态。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  ToolResult execute(const Json & arguments) override;
  void shutdown() noexcept override {}

private:
  std::shared_ptr<NavigationRuntime> navigation_;
  std::shared_ptr<SharedResources> resources_;
  std::string rgbd_resource_;
};
}  // namespace vln_agent
