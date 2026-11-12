#pragma once

#include <memory>
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;

/** @brief 将同一环境图中的有序像素目标转换后交给 NavigateThroughPoses。 */
class NavigateThroughPixelsStartNavigation : public ToolPluginBase
{
public:
  /** @brief 复用本组导航资源，像素数上限来自 group_settings.max_pixel_goals。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 接受 environment_id 和 pixels，返回 task_id 及转换后的 goals。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 共享资源由管理器统一关闭。 */
  void shutdown() noexcept override {}

private:
  std::shared_ptr<NavigationRuntime> navigation_;  ///< 与导航、截图和环顾共用任务槽与快照。
};
}  // namespace vln_agent
