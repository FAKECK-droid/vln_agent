#include "vln_agent/utils/json.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <yaml-cpp/yaml.h>

#include "vln_agent/utils/time.hpp"

namespace vln_agent::utils
{

namespace
{
/** @brief 在追加输入前检查图片描述；系统提示词和无图片的原生项直接传递。 */
void validateNativeImages(const Json & input)
{
  const auto key = input.value("type", std::string()) == "function_call_output" ?
    "output" : "content";
  const auto content = input.find(key);
  if (content == input.end() || !content->is_array()) {return;}
  const auto count = std::count_if(content->begin(), content->end(), [](const Json & part) {
      return part.is_object() && part.value("type", std::string()) == "input_image";
    });
  if (count == 0) {return;}
  if (content->empty() || content->at(0).value("type", std::string()) != "input_text") {
    throw std::invalid_argument("带图片的输入需要首个 input_text 提供业务 JSON 与 images 元数据");
  }
  validateImageMetadata(Json::parse(content->at(0).at("text").get<std::string>()), count);
}
}  // namespace

Json businessPayload(const std::string & source, Json data)
{
  return {{"source", source}, {"time_stamp", utcNow()}, {"data", std::move(data)}};
}

Json posePayload(const std::string & frame_id, double x, double y, double yaw,
  const std::string & time_stamp)
{
  Json pose{{"frame_id", frame_id}, {"x", x}, {"y", y}, {"yaw", yaw}};
  if (!time_stamp.empty()) {pose["time_stamp"] = time_stamp;}
  return pose;
}

void validateImageMetadata(const Json & payload, std::size_t image_count)
{
  const auto descriptions = payload.find("images");
  if (descriptions == payload.end()) {
    if (image_count == 0) {return;}
    throw std::invalid_argument("带图片的输入需要 images 元数据数组");
  }
  if (!descriptions->is_array() || descriptions->size() != image_count ||
    !std::all_of(descriptions->begin(), descriptions->end(), [](const Json & image) {
      return image.is_object();
    }))
  {
    throw std::invalid_argument("images 须按图片顺序提供等量的元数据对象");
  }
}

Json yamlToJson(const YAML::Node & node)
{
  if (node.IsNull()) {
    return nullptr;
  }
  if (node.IsMap()) {
    Json value = Json::object();
    for (const auto & item : node) {
      value[item.first.as<std::string>()] = yamlToJson(item.second);
    }
    return value;
  }
  if (node.IsSequence()) {
    Json value = Json::array();
    for (const auto & item : node) {
      value.push_back(yamlToJson(item));
    }
    return value;
  }
  const auto text = node.as<std::string>();
  const auto scalar = Json::parse(text, nullptr, false);
  return node.Tag() != "!" && !scalar.is_discarded() && scalar.is_primitive() ?
         scalar : Json(text);
}

Json settingsOf(const YAML::Node & node)
{
  const auto settings = node["settings"] ? yamlToJson(node["settings"]) : Json::object();
  if (!settings.is_object()) {
    throw std::invalid_argument("settings 必须为对象");
  }
  return settings;
}

void appendInputs(Json & destination, const Json::array_t & input)
{
  for (const auto & item : input) {
    if (!item.is_object()) {
      throw std::invalid_argument("每个 input 项必须为原生 API 对象");
    }
    validateNativeImages(item);
  }
  auto & items = destination.get_ref<Json::array_t &>();
  items.insert(items.end(), input.begin(), input.end());
}

void appendInputs(Json & destination, const Json & input)
{
  if (input.is_array()) {
    appendInputs(destination, input.get_ref<const Json::array_t &>());
  } else if (input.is_object()) {
    validateNativeImages(input);
    destination.push_back(input);
  } else {
    throw std::invalid_argument("input 必须为对象或对象数组");
  }
}

void prependArray(Json & destination, const Json & input)
{
  auto & items = destination.get_ref<Json::array_t &>();
  const auto & prefix = input.get_ref<const Json::array_t &>();
  items.insert(items.begin(), prefix.begin(), prefix.end());
}

Json functionCallOutput(
  const std::string & call_id, const Json & payload, const std::vector<Json> & images)
{
  validateImageMetadata(payload, images.size());
  Json content = Json::array({{{"type", "input_text"}, {"text", payload.dump()}}});
  for (const auto & image : images) {
    content.push_back(image);
  }
  return {{"type", "function_call_output"}, {"call_id", call_id}, {"output", content}};
}

}  // namespace vln_agent::utils
