#pragma once

#include <memory>

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{

class NavigationRuntime;

/** @brief 查询共享导航任务的最新反馈。 */
class NavigateToPoseGetFeedback : public ToolPluginBase
{
public:
  /** @brief 取得本组共用的 NavigationRuntime，函数配置为空。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 将本次参数转交共享导航资源。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 无独立等待；共享资源由 SharedResources 统一关闭。 */
  void shutdown() noexcept override;

private:
  std::shared_ptr<NavigationRuntime> navigation_;  ///< 本组共享导航资源。
};

}  // namespace vln_agent
