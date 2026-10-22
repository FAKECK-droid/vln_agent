#include "vln_agent/utils/json.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace vln_agent::utils
{

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
  }
  auto & items = destination.get_ref<Json::array_t &>();
  items.insert(items.end(), input.begin(), input.end());
}

void appendInputs(Json & destination, const Json & input)
{
  if (input.is_array()) {
    appendInputs(destination, input.get_ref<const Json::array_t &>());
  } else if (input.is_object()) {
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
  Json content = Json::array({{{"type", "input_text"}, {"text", payload.dump()}}});
  for (const auto & image : images) {
    content.push_back(image);
  }
  return {{"type", "function_call_output"}, {"call_id", call_id}, {"output", content}};
}

}  // namespace vln_agent::utils
