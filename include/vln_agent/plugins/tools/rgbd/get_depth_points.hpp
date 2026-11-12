#pragma once

#include <memory>
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class RgbdRuntime;

/** @brief 批量查询同一 pair 的深度像素，按输入顺序返回逐点投影结果。 */
class GetDepthPoints : public ToolPluginBase
{
public:
  /** @brief 取得同组共享的图像对、标定和 TF 资源。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 接受 image_pair_id、pixels 数组及 target_frame，复用一次批量投影。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 共享资源由注册表统一关闭。 */
  void shutdown() noexcept override {}

private:
  std::shared_ptr<RgbdRuntime> rgbd_;  ///< 与 capture_pair 和单点工具共享有界缓存。
};
}  // namespace vln_agent
