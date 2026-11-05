#pragma once

#include <memory>
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;

/** @brief 使用环境快照中的像素位置和相对图片的朝向启动导航。 */
class NavigateToPixelStartNavigation : public ToolPluginBase
{
public:
  /** @brief 取得本组共用的导航资源；服务和绘图配置来自 group_settings。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /**
   * @brief 使用环境快照中的像素位置和相对图片的朝向启动导航。
   * @param arguments 包含 environment_id、pixel_x、pixel_y 和 yaw；yaw 为弧度，0 朝图片上方，正值逆时针朝左。
   * @return 本工具的导航启动结果、目标信息和 task_id，不另发内部函数结果。
   */
  ToolResult execute(const Json & arguments) override;
  /** @brief 插件无独立资源；共享资源由 SharedResources 统一关闭。 */
  void shutdown() noexcept override {}

private:
  std::shared_ptr<NavigationRuntime> navigation_;  ///< 本组共用的导航、TF 与环境快照。
};
}  // namespace vln_agent
