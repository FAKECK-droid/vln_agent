#pragma once

#include <memory>
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;

/** @brief 读取代价地图并返回机器人朝上的环境图片。 */
class GetSurroundings : public ToolPluginBase
{
public:
  /** @brief 取得本组共用的导航资源；服务和绘图配置来自 group_settings。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /**
   * @brief 读取代价地图并返回机器人朝上的环境图片。
   * @param arguments 包含 costmap（local/global）和 range_m（正方形边长，米）。
   * @return 环境 ID、图片几何信息与原生 PNG 内容项。
   */
  ToolResult execute(const Json & arguments) override;
  /** @brief 插件无独立资源；共享资源由 SharedResources 统一关闭。 */
  void shutdown() noexcept override {}

private:
  std::shared_ptr<NavigationRuntime> navigation_;  ///< 本组共用的导航、TF 与环境快照。
};
}  // namespace vln_agent
