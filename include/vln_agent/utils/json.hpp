#pragma once

#include <cstddef>
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
/**
 * @brief 创建用户、事件和工具共用的业务载荷，填写 source、time_stamp 和 data。
 * @param source 信息来源，user 或 robot。
 * @param data 本条信息的具体载荷，默认空对象。
 * @return 公共字段对象；instruction、event、origin、ok 等由提供方填写。
 */
Json businessPayload(const std::string & source, Json data = Json::object());
/**
 * @brief 构造带坐标系的二维小车位姿。
 * @param frame_id 位姿所在坐标系。
 * @param x 米制 x 坐标。
 * @param y 米制 y 坐标。
 * @param yaw 逆时针弧度角。
 * @param time_stamp 可选的真实 UTC 采样时间。
 * @return 可用于公共 pose 字段的对象。
 */
Json posePayload(const std::string & frame_id, double x, double y, double yaw,
  const std::string & time_stamp = "");
/** @brief 检查 images 元数据数组与本条消息的原生图片数量逐一对应。 */
void validateImageMetadata(const Json & payload, std::size_t image_count);
/** @brief 追加原生输入；含图片的业务输入须提供顺序对应的 images，数组整体检查后追加。 */
void appendInputs(Json & destination, const Json & input);
/** @brief 追加输入对象数组，不形成嵌套数组。 */
void appendInputs(Json & destination, const Json::array_t & input);
/** @brief 将一个数组插入另一个数组的头部，用于恢复未完成输入。 */
void prependArray(Json & destination, const Json & input);
/** @brief 将统一业务载荷和对应图片包装为 function_call_output，填写 call_id。 */
Json functionCallOutput(
  const std::string & call_id, const Json & payload, const std::vector<Json> & images);
}  // namespace utils
}  // namespace vln_agent
