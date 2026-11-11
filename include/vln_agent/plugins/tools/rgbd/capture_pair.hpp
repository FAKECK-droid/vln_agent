#pragma once

#include <memory>

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class RgbdRuntime;

/** @brief 保存 RGB-D 图像对，向模型仅返回彩色图与图像对 ID。 */
class CapturePair : public ToolPluginBase
{
public:
  /** @brief 读取 image_scale 设置，取得同组共享的图像、标定和 TF 资源。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 将模型参数交给共享资源，不依赖其他组件。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 无独立资源；由 SharedResources 统一关闭。 */
  void shutdown() noexcept override {}

private:
  double image_scale_{1.0};                ///< 返回彩色图的等比例缩放因子。
  std::shared_ptr<RgbdRuntime> rgbd_;        ///< 同组插件持有同一资源。
};
}  // namespace vln_agent
