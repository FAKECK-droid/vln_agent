#pragma once

#include <string>
#include <vector>

#include "vln_agent/tools/tool_context.hpp"

namespace vln_agent
{

/** @brief 单次函数的业务结果；API 外壳和 call_id 由管理器填写。 */
struct ToolResult
{
  Json payload;                            ///< 至少包含布尔字段 ok 的结果对象。
  std::vector<Json> images;                 ///< 可选的原生 input_image 内容项。

  /** @brief 构造带错误码和说明的失败结果。 */
  static ToolResult failure(const std::string & code, const std::string & message)
  {
    return {Json{{"ok", false}, {"code", code}, {"message", message}}, {}};
  }
};

/** @brief 每个模型 function 对应一个持久的 pluginlib 插件实例。 */
class ToolPluginBase
{
public:
  virtual ~ToolPluginBase() = default;

  /** @brief 注入公共依赖和本函数配置；在执行线程启动前调用一次。 */
  virtual void initialize(const ToolContext & context, const Json & settings) = 0;

  /**
   * @brief 执行单次调用；非阻塞配置下须允许同一实例并发执行。
   * @param arguments 从 function_call.arguments 解析得到的参数对象。
   * @return 本次函数结果，不包含普通业务输入外壳。
   */
  virtual ToolResult execute(const Json & arguments) = 0;

  /** @brief 唤醒执行等待，允许与 execute 并发且可重复调用，不抛异常。 */
  virtual void shutdown() noexcept = 0;
};

}  // namespace vln_agent
