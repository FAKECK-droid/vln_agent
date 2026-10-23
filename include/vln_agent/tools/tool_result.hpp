#pragma once

#include <string>
#include <vector>

#include "vln_agent/utils/json.hpp"
#include "vln_agent/utils/time.hpp"

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
    return {Json{{"ok", false}, {"code", code}, {"message", message},
        {"time_stamp", utils::utcNow()}}, {}};
  }
};

}  // namespace vln_agent
