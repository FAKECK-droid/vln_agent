#pragma once

#include <cstdint>
#include <memory>

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class RgbdRuntime;

/** @brief 读取未交付的历史和彩色最新图，并缓存各帧深度。 */
class CapturePairSequence : public ToolPluginBase
{
public:
  /** @brief 装载每批上限和缩放设置，复用同组 RGB-D 资源。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 原子读取并推进当前工具的历史游标。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 共享资源由组统一关闭。 */
  void shutdown() noexcept override {}

private:
  double image_scale_{0.5};
  std::uint32_t max_batch_pairs_{8};
  std::shared_ptr<RgbdRuntime> rgbd_;
};
}  // namespace vln_agent
