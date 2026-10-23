#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent::test
{

/** @brief 测试专用执行门闩，用显式释放验证调度，不依赖执行耗时猜测。 */
struct Probe
{
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<std::string> started;
  std::unordered_set<std::string> released;
  int active{0};
  int peak{0};
  int resource_shutdowns{0};
};

/** @brief 仅测试动态库使用的观测对象，生产插件不依赖它。 */
Probe & probe();

/** @brief 用于验证组内共享和管理器隔离的测试资源。 */
class Token : public SharedResource
{
public:
  Token();
  void shutdown() noexcept override;
  int id;                                  ///< 本进程内的测试资源编号。

private:
  bool closed_{false};
};

/** @brief 接受等待门闩、异常及图片参数的测试插件。 */
class TestTool : public ToolPluginBase
{
public:
  void initialize(const ToolContext & context, const Json & settings) override;
  ToolResult execute(const Json & arguments) override;
  void shutdown() noexcept override;

private:
  std::shared_ptr<const std::atomic_bool> stopping_;
  std::shared_ptr<Token> token_;
  Json settings_;
  Json group_settings_;
};

}  // namespace vln_agent::test
