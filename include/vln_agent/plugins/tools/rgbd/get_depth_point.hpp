#pragma once

#include <memory>

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class RgbdRuntime;

/** @brief 读取缓存深度像素，返回指定坐标系中的三维坐标。 */
class GetDepthPoint : public ToolPluginBase
{
public:
  /** @brief 取得 rgbd 工具组共享的图像、标定和 TF 资源。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 将模型参数交给共享资源，不依赖其他组件。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 无独立资源；由 SharedResources 统一关闭。 */
  void shutdown() noexcept override {}

private:
  std::shared_ptr<RgbdRuntime> rgbd_;        ///< 同组插件持有同一资源。
};
}  // namespace vln_agent
