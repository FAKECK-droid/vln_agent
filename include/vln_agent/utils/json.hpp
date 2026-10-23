#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace YAML {class Node;}

namespace vln_agent
{

using Json = nlohmann::json;

namespace utils
{
/** @brief 将 YAML 转为 JSON，保留显式引号字符串及常用标量类型。 */
Json yamlToJson(const YAML::Node & node);
/** @brief 读取可省略的 settings 对象，省略时返回空对象。 */
Json settingsOf(const YAML::Node & node);
/** @brief 追加一个原生输入对象或输入数组；数组先整体检查，再按顺序追加。 */
void appendInputs(Json & destination, const Json & input);
/** @brief 追加输入对象数组，不形成嵌套数组。 */
void appendInputs(Json & destination, const Json::array_t & input);
/** @brief 将一个数组插入另一个数组的头部，用于恢复未完成输入。 */
void prependArray(Json & destination, const Json & input);
/** @brief 包装一个函数结果；保留原生图片项，不添加普通业务输入字段。 */
Json functionCallOutput(
  const std::string & call_id, const Json & payload, const std::vector<Json> & images);
}  // namespace utils
}  // namespace vln_agent
