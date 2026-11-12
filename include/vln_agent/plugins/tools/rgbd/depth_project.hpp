#pragma once
#include <memory>
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class RgbdRuntime;
/** @brief 对同一图像对的多个实例执行分割或正点深度投影。 */
class DepthProject : public ToolPluginBase
{
public:
  /** @brief 取得同组 RGB-D 共享资源。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 返回各实例有效深度的三维均值。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 生命周期由共享资源管理。 */
  void shutdown() noexcept override {}
private:
  std::shared_ptr<RgbdRuntime> rgbd_;  ///< 同组图像、标定、TF 和分割资源。
};
}  // namespace vln_agent
