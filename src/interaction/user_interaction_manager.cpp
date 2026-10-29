#include "vln_agent/interaction/user_interaction_manager.hpp"

#include <algorithm>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "vln_agent/interaction/user_interface_base.hpp"
#include "vln_agent/session/session_manager.hpp"
#include "vln_agent/tools/tool_manager.hpp"
#include "vln_agent/utils/time.hpp"

namespace vln_agent
{
namespace
{
const std::vector<std::string> fields{
  "request", "response.messages", "response.function_calls", "response.tool_outputs",
  "response.stream", "tools.definitions", "tools.status", "tools.queue",
  "tools.finished_calls", "session.request_status", "session.context_status"};
const std::vector<std::string> state_fields{
  "request", "session.request_status", "session.context_status", "response.stream",
  "tools.status", "tools.queue"};

const Json * findPath(const Json & object, const std::string & path)
{
  const Json * value = &object;
  std::size_t start = 0;
  while (true) {
    const auto dot = path.find('.', start);
    const auto key = path.substr(start, dot == std::string::npos ? dot : dot - start);
    if (!value->is_object() || !value->contains(key)) {return nullptr;}
    value = &value->at(key);
    if (dot == std::string::npos) {return value;}
    start = dot + 1;
  }
}

void putPath(Json & object, const std::string & path, Json value)
{
  const auto dot = path.find('.');
  if (dot == std::string::npos) {object[path] = std::move(value);}
  else {object[path.substr(0, dot)][path.substr(dot + 1)] = std::move(value);}
}

struct Options
{
  Json enabled = Json::object();
  bool images{false};

  bool has(const std::string & key) const {return enabled.value(key, false);}
  std::string key() const {return enabled.dump() + (images ? ":images" : ":no-images");}
};

Options optionsOf(const YAML::Node & channel)
{
  const auto config = channel["transmit"] ? utils::yamlToJson(channel["transmit"]) : Json::object();
  Options options;
  for (const auto & path : fields) {
    const auto value = findPath(config, path);
    options.enabled[path] = value ? value->get<bool>() : false;
  }
  options.images = channel["include_images"].as<bool>(false);
  return options;
}

/** @brief 只处理展示副本；原生请求、响应及其他插件的数据保持原样。 */
Json displayCopy(const Json & source, bool images)
{
  if (source.is_array()) {
    Json result = Json::array();
    for (const auto & item : source) {result.push_back(displayCopy(item, images));}
    return result;
  }
  if (!source.is_object()) {return source;}
  const auto type = source.contains("type") && source.at("type").is_string() ?
    source.at("type").get<std::string>() : std::string();
  if (!images && (type == "input_image" || type == "output_image" ||
    type == "image" || type == "image_generation_call"))
  {
    return {{"type", type}, {"omitted", true}};
  }
  static const std::map<std::string, std::string> timestamps{
    {"created_at", "created_time_stamp"}, {"completed_at", "completed_time_stamp"},
    {"expires_at", "expires_time_stamp"}, {"failed_at", "failed_time_stamp"},
    {"cancelled_at", "cancelled_time_stamp"}};
  Json result = Json::object();
  for (auto it = source.begin(); it != source.end(); ++it) {
    const auto stamp = timestamps.find(it.key());
    if (stamp != timestamps.end() && it.value().is_number()) {
      result[stamp->second] = utils::formatUnixSeconds(it.value().get<double>());
    } else {
      result[it.key()] = displayCopy(it.value(), images);
    }
  }
  return result;
}

/** @brief 每次交付完整文本；已有客户端按相同位置执行 replace。 */
void appendText(Json & messages, const std::string & response_id,
                std::size_t output, std::size_t content, const std::string & type,
                const std::string & text)
{
  messages.push_back({
    {"response_id", response_id}, {"output_index", output}, {"content_index", content},
    {"type", type}, {"op", "replace"}, {"text", text}});
}
}  // namespace

struct UserInteractionManager::Impl
{
  struct Channel {std::shared_ptr<UserInterfaceBase> plugin; std::size_t profile;};
  struct Batch
  {
    std::vector<std::shared_ptr<const Json>> outgoing;
    std::vector<bool> accepted;
  };

  Impl(const std::shared_ptr<rclcpp::Node> & node_ref, UserInteractionSources refs)
  : node(node_ref), sources(std::move(refs)) {}
  ~Impl() {shutdown();}

  bool wants(const std::string & field) const
  {
    return std::any_of(profiles.begin(), profiles.end(),
      [&](const Options & p) {return p.has(field);});
  }

  void load(const std::string & path)
  {
    if (loaded || closed) {throw std::logic_error("交互管理器已装载或关闭");}
    auto owner = node.lock();
    if (!owner) {throw std::logic_error("用户交互 ROS 节点已失效");}
    const auto config = YAML::LoadFile(path)["user_interaction"];
    if (!config.IsMap() || !config["channels"].IsSequence()) {
      throw std::invalid_argument("缺少 user_interaction.channels 数组");
    }
    try {
      loader = std::make_unique<pluginlib::ClassLoader<UserInterfaceBase>>(
        "vln_agent", "vln_agent::UserInterfaceBase");
      for (const auto & item : config["channels"]) {
        const auto options = optionsOf(item);
        auto profile = std::find_if(profiles.begin(), profiles.end(),
          [&](const Options & p) {return p.key() == options.key();});
        const auto index = static_cast<std::size_t>(std::distance(profiles.begin(), profile));
        if (profile == profiles.end()) {profiles.push_back(options);}
        auto plugin = loader->createSharedInstance(item["plugin"].as<std::string>());
        channels.push_back({plugin, index});
        plugin->initialize(owner, utils::settingsOf(item));
      }
      if (wants("tools.definitions")) {
        auto tools = sources.tools.lock();
        if (!tools) {throw std::logic_error("工具定义查询需要有效的 ToolManager");}
        definitions = tools->getTools();  // 全部实例共用，仅查询一次。
      }
      loaded = true;
    } catch (...) {
      for (auto & channel : channels) {channel.plugin->shutdown();}
      channels.clear();
      profiles.clear();
      loader.reset();
      throw;
    }
  }

  std::unique_ptr<Batch> prepare(const Json & response, const Json & outputs)
  {
    if (!outputs.is_array()) {throw std::invalid_argument("tool_outputs 必须为数组");}
    if (!response.is_null() && (!response.is_object() ||
      response.at("status") != "completed" || !response.at("output").is_array()))
    {
      throw std::invalid_argument("new_response 必须为新增的完整 completed response");
    }
    auto batch = std::make_unique<Batch>();
    batch->accepted.resize(channels.size(), false);
    Json snapshot = Json::object();
    auto session = sources.session.lock();
    const bool need_session = wants("request") || wants("session.request_status") ||
      wants("session.context_status") || wants("response.stream");
    if (need_session && !session) {
      throw std::logic_error("所选字段需要有效的 SessionManager");
    }
    if (wants("request")) {
      auto request = session->getCurrentRequest();
      if (request.is_object()) {request.erase("tools");}
      snapshot["request"] = std::move(request);
    }
    if (wants("session.request_status")) {
      snapshot["session"]["request_status"] = session->getRequestStatus();
    }
    if (wants("session.context_status")) {
      snapshot["session"]["context_status"] = session->getContextStatus();
    }
    Json stream;
    if (wants("response.stream")) {
      stream = session->getStreamSnapshot();
      snapshot["response"]["stream"] = stream.is_null() ? Json() :
        Json{{"response_id", stream.at("response_id")}, {"state", stream.at("state")}};
    }

    auto tools = sources.tools.lock();
    if ((wants("tools.status") || wants("tools.queue")) && !tools) {
      throw std::logic_error("所选字段需要有效的 ToolManager");
    }
    if (wants("tools.status")) {snapshot["tools"]["status"] = tools->getStatus();}
    if (wants("tools.queue")) {snapshot["tools"]["queue"] = tools->getQueueSnapshot();}

    // 完成事件由 Agent 本次传入，不向工具管理器查询已删除的历史记录。
    Json terminal = Json::array();
    for (const auto & item : outputs) {
      if (item.at("type") != "function_call_output") {
        throw std::invalid_argument("tool_outputs 元素必须为 function_call_output");
      }
      const auto id = item.at("call_id").get<std::string>();
      if (wants("tools.finished_calls")) {
        terminal.push_back({{"call_id", id}, {"state", "finished"}});
      }
    }

    const bool has_response = !response.is_null();
    const auto response_id = has_response ? response.at("id").get<std::string>() : std::string();
    const auto stream_id = stream.is_null() ? std::string() :
      stream.at("response_id").get<std::string>();
    const auto stamp = utils::utcNow();
    for (const auto & options : profiles) {
      Json packet = Json::object();
      for (const auto & path : state_fields) {
        if (!options.has(path)) {continue;}
        const auto current = findPath(snapshot, path);
        if (!current) {continue;}
        putPath(packet, path, displayCopy(*current, options.images));
      }
      if (options.has("tools.definitions")) {
        packet["tools"]["definitions"] = definitions;
      }

      Json messages = Json::array();
      if (options.has("response.messages")) {
        if (has_response) {
          const auto & output = response.at("output");
          for (std::size_t i = 0; i < output.size(); ++i) {
            if (output[i].value("type", std::string()) != "message") {continue;}
            const auto & content = output[i].at("content");
            for (std::size_t j = 0; j < content.size(); ++j) {
              const auto type = content[j].value("type", std::string());
              if (type == "output_text" || type == "refusal") {
                appendText(messages, response_id, i, j, type,
                  content[j].at(type == "refusal" ? "refusal" : "text").get<std::string>());
              }
            }
          }
        }
        // 同轮的完整结果优先；可同时显示下一轮已经开始的流式文本。
        if (options.has("response.stream") && !stream_id.empty() &&
          (!has_response || stream_id != response_id))
        {
          for (const auto & part : stream.at("content")) {
            appendText(messages, stream_id, part.at("output_index").get<std::size_t>(),
              part.at("content_index").get<std::size_t>(), part.at("type").get<std::string>(),
              part.at("text").get<std::string>());
          }
        }
      }
      if (options.has("response.messages")) {packet["response"]["messages"] = std::move(messages);}
      if (has_response && (options.has("response.messages") ||
        options.has("response.function_calls") || options.has("response.stream")))
      {
        packet["response"]["id"] = response_id;
        packet["response"]["status"] = response.at("status");
      }
      if (options.has("response.function_calls")) {
        Json calls = Json::array();
        if (has_response) {
          for (const auto & item : response.at("output")) {
            if (item.value("type", std::string()) == "function_call") {calls.push_back(item);}
          }
        }
        packet["response"]["function_calls"] = std::move(calls);
      }
      if (options.has("response.tool_outputs")) {
        packet["response"]["tool_outputs"] = displayCopy(outputs, options.images);
      }
      if (options.has("tools.finished_calls")) {
        packet["tools"]["finished_calls"] = terminal;
      }
      if (packet.empty()) {batch->outgoing.push_back(nullptr);}
      else {
        packet["type"] = "interaction_update";
        packet["batch_id"] = next_batch;
        packet["time_stamp"] = stamp;
        packet["full"] = next_batch == 1;  // 兼容现有客户端：仅首次包标记新基线。
        batch->outgoing.push_back(std::make_shared<const Json>(std::move(packet)));
      }
    }
    return batch;
  }

  Json exchange(const Json & response, const Json & outputs)
  {
    if (!loaded || closed) {throw std::logic_error("交互管理器未装载或已经关闭");}
    if (!pending) {pending = prepare(response, outputs);}
    for (std::size_t i = 0; i < channels.size(); ++i) {
      const auto & channel = channels[i];
      const auto outgoing = pending->accepted[i] ? nullptr :
        pending->outgoing.at(channel.profile);
      auto inputs = channel.plugin->exchange(outgoing);
      pending->accepted[i] = true;
      if (!inputs.is_array()) {
        throw std::invalid_argument("接口 exchange() 必须返回原生输入数组");
      }
      utils::appendInputs(input_buffer, inputs);
    }
    pending.reset();
    ++next_batch;
    Json inputs = Json::array();
    inputs.swap(input_buffer);
    return inputs;
  }

  void shutdown() noexcept
  {
    if (closed) {return;}
    closed = true;
    for (auto & channel : channels) {channel.plugin->shutdown();}
    channels.clear();
    pending.reset();
    profiles.clear();
    input_buffer = Json::array();
    definitions = Json::array();
    loader.reset();
  }

  std::weak_ptr<rclcpp::Node> node;
  UserInteractionSources sources;
  std::unique_ptr<pluginlib::ClassLoader<UserInterfaceBase>> loader;
  std::vector<Options> profiles;
  std::vector<Channel> channels;
  std::unique_ptr<Batch> pending;
  Json definitions = Json::array(), input_buffer = Json::array();
  std::uint64_t next_batch{1};
  bool loaded{false}, closed{false};
};

UserInteractionManager::UserInteractionManager(
  const std::shared_ptr<rclcpp::Node> & node, const UserInteractionSources & sources)
: impl_(std::make_unique<Impl>(node, sources)) {}
UserInteractionManager::~UserInteractionManager() = default;
void UserInteractionManager::load(const std::string & path) {impl_->load(path);}
Json UserInteractionManager::exchange(const Json & response, const Json & outputs)
{
  return impl_->exchange(response, outputs);
}
void UserInteractionManager::shutdown() {impl_->shutdown();}
}  // namespace vln_agent
