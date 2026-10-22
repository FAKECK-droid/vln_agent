#pragma once

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "vln_agent/tools/tool_context.hpp"

namespace vln_agent
{

/** @brief 可独立取得结果的单次函数调用句柄。 */
struct ToolCallHandle
{
  std::string call_id;                      ///< 模型提供的调用关联标志。
  std::string name;                         ///< 原生 function 名称。
  std::shared_future<Json> result;          ///< 单个 function_call_output 输入项。
};

/** @brief 加载工具插件并按 FIFO、容量及阻塞策略执行调用。 */
class ToolManager
{
public:
  /**
   * @brief 复用外部节点，管理器不持有节点所有权或启动 ROS 执行器。
   * @param node 须保持存活，且执行器应运行到 shutdown 返回。
   */
  explicit ToolManager(const rclcpp::Node::SharedPtr & node);
  ~ToolManager();

  /**
   * @brief 一次性加载全局 YAML 的 tool_manager 段，失败时不发布部分工具。
   * @param global_yaml_path 配置路径；定义文件相对于该文件所在目录解析。
   * @throws std::exception 配置、文件或插件装载失败，不支持热更新。
   */
  void load(const std::string & global_yaml_path);

  /** @brief 按组及函数配置顺序返回启用函数的原生定义数组。 */
  Json getTools() const;

  /**
   * @brief 只提交完整响应中的 function_call，不等待执行结果。
   * @param response status 为 completed 的完整 Responses 响应。
   * @return 按输出顺序排列的调用句柄；同一 call_id 复用原 future。
   * @throws std::exception 响应结构错误或没有可关联的 call_id/name。
   * @note 调用错误通过各自 future 返回；去重记录保留到管理器销毁。
   */
  std::vector<ToolCallHandle> submit(const Json & response);

  /** @brief 停止接收，完成未执行调用，唤醒资源并回收全部线程；可重复调用。 */
  void shutdown();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;              ///< 隐藏插件加载及线程调度细节。
};

}  // namespace vln_agent
