#pragma once

#include <string>
#include <utility>
#include <vector>

#include "vln_agent/utils/json.hpp"

namespace vln_agent
{

/** @brief 单次函数的业务结果；API 外壳和 call_id 由管理器填写。 */
struct ToolResult
{
  Json payload;                            ///< source/time_stamp/ok/data，按需包含 pose 和 images。
  std::vector<Json> images;                 ///< 可选的原生 input_image 内容项。

  /** @brief 构造成功结果；ok 与 data 同级，具体函数载荷放入 data。 */
  static ToolResult success(Json data = Json::object())
  {
    auto payload = utils::businessPayload("robot", std::move(data));
    payload["ok"] = true;
    return {std::move(payload), {}};
  }

  /** @brief 构造失败结果；code/message 与 data 同级，data 默认空对象。 */
  static ToolResult failure(const std::string & code, const std::string & message)
  {
    auto payload = utils::businessPayload("robot");
    payload["ok"] = false;
    payload["code"] = code;
    payload["message"] = message;
    return {std::move(payload), {}};
  }
};

}  // namespace vln_agent
