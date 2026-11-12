#pragma once
#include <memory>
#include <string>
#include "vln_agent/plugins/tools/navigation/goal_evaluation.hpp"
#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
class NavigationRuntime;
/** @brief 从图像目标生成接近位置，评估后启动共享导航。 */
class NavigateToImageTarget : public ToolPluginBase
{
public:
  /** @brief 取得导航资源及 RGB-D 资源地址，读取评分权重。 */
  void initialize(const ToolContext & context, const Json & settings) override;
  /** @brief 仅查询缓存 pair，直接复用投影和导航资源。 */
  ToolResult execute(const Json & arguments) override;
  /** @brief 共享资源由注册表统一关闭。 */
  void shutdown() noexcept override {}
private:
  std::shared_ptr<NavigationRuntime> navigation_;  ///< 复用同组导航状态、TF 和客户端。
  std::shared_ptr<SharedResources> resources_;    ///< 执行时查找已经载入的 RGB-D 资源。
  std::string rgbd_resource_;                    ///< 缺省 rgbd/rgbd，支持自定义组名。
  navigation::GoalScoring scoring_;              ///< 本函数独立的选择权重。
};
}  // namespace vln_agent
