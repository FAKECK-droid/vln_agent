#pragma once

#include <memory>
#include <string>
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;

/** @brief 复用环视实例投影，按实例顺序启动单点或多点物体导航。 */
class NavigateToObjectsSurround : public ToolPluginBase
{
public:
  /** @brief 共用本组导航状态，保存 RGB-D 资源名供执行时查询。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 一次投影全部实例，向 Nav2 提交一个完整导航目标。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 共享资源由注册表统一关闭。 */
  void shutdown() noexcept override {}

private:
  std::shared_ptr<SharedResources> resources_;   ///< 只查询已有资源，避免工具组装载顺序依赖。
  std::shared_ptr<NavigationRuntime> navigation_; ///< 与其他导航函数共用任务槽。
  std::string rgbd_resource_;                   ///< 所引用的 RGB-D 资源，默认 rgbd/rgbd。
};
}  // namespace vln_agent
