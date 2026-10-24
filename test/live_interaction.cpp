// 显式运行的真实 API 集成测试；仅连接用户配置的局域网反代，固定 gpt-6-luna。
#include <chrono>
#include <fstream>
#include <filesystem>
#include <vector>
#include <string>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <thread>
#include <sys/stat.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include "vln_agent/interaction/user_interaction_manager.hpp"
#include "vln_agent/session/session_manager.hpp"
#include "vln_agent/tools/tool_manager.hpp"
#include "vln_agent/utils/time.hpp"

using namespace std::chrono_literals;
using vln_agent::Json;

static void require(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

static Json userInput(const std::string & instruction)
{
  return {{"role", "user"}, {"content", Json::array({Json{
    {"type", "input_text"}, {"text", Json{{"source", "user"},
      {"time_stamp", vln_agent::utils::utcNow()}, {"instruction", instruction}}.dump()}}})}};
}

static std::string textOf(const Json & response)
{
  std::string text;
  for (const auto & item : response.at("output")) {
    if (item.value("type", std::string()) != "message") {continue;}
    for (const auto & content : item.at("content")) {
      if (content.value("type", std::string()) == "output_text") {
        text += content.at("text").get<std::string>();
      }
    }
  }
  return text;
}

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: live_interaction_test CONFIG_YAML OUTPUT_FIXTURE_JSON\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  try {
    const auto result_path = std::filesystem::absolute(argv[2]);
    std::filesystem::create_directories(result_path.parent_path());
    auto config = YAML::LoadFile(argv[1]);
    auto session_config = config["session_manager"];
    const auto endpoint = session_config["base_url"].as<std::string>();
    // 此程序只允许回环或 RFC1918 局域网反代；不会直接连接云端域名。
    const bool local = endpoint.find("127.0.0.1") != std::string::npos ||
      endpoint.find("localhost") != std::string::npos ||
      endpoint.find("192.168.") != std::string::npos ||
      endpoint.find("10.") != std::string::npos || endpoint.find("172.") != std::string::npos;
    require(local, "真实测试必须配置用户的回环/局域网反代地址");
    session_config["model"] = "gpt-6-luna";
    session_config["store"] = true;
    session_config["stream_output"] = true;
    session_config["reasoning_effort"] = "low";
    session_config["max_output_tokens"] = 2048;
    session_config["request_timeout_sec"] = 120;
    session_config["max_request_hz"] = 10;
    const auto definition_path = result_path.parent_path() / "live_tool.json";
    const Json definition{{"type", "function"}, {"name", "interaction_test_echo"},
      {"description", "受控接口测试：立即返回成功，不访问机器人或外部系统。"},
      {"async", true}, {"strict", true}, {"parameters", {
        {"type", "object"}, {"properties", Json::object()}, {"required", Json::array()},
        {"additionalProperties", false}}}};
    std::ofstream(definition_path) << Json::array({definition}).dump(2);
    const Json test_function{{"name", "interaction_test_echo"},
      {"plugin", "vln_agent_test/TestTool"}, {"blocking", false}};
    const Json test_group{{"id", "test"}, {"definition", definition_path.string()},
      {"functions", Json::array({test_function})}};
    config["tool_manager"] = YAML::Load(
      Json{{"groups", Json::array({test_group})}}.dump());
    const std::string send_topic = "/vln_live_test/output";
    const std::string receive_topic = "/vln_live_test/input";
    config["user_interaction"] = YAML::Load(Json{{"channels", Json::array({Json{
      {"id", "ros"}, {"plugin", "vln_agent/RosTopicInterface"}, {"include_images", false},
      {"transmit", {{"request", true}, {"response", {{"messages", true},
        {"function_calls", true}, {"tool_outputs", true}, {"stream", true}}},
        {"tools", {{"definitions", true}, {"status", true}, {"queue", true},
          {"finished_calls", true}}},
        {"session", {{"request_status", true}, {"context_status", true}}}}},
      {"settings", {{"send_topic", send_topic}, {"receive_topic", receive_topic}}}}})}}.dump());
    const auto runtime_config = result_path.parent_path() / "live_runtime.local.yaml";
    {std::ofstream(runtime_config) << config;}
    chmod(runtime_config.c_str(), 0600);

    auto node = std::make_shared<rclcpp::Node>("live_interaction_test");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::vector<Json> packets;
    auto subscription = node->create_subscription<std_msgs::msg::String>(
      send_topic, 10, [&](std_msgs::msg::String::ConstSharedPtr message) {
        packets.push_back(Json::parse(message->data));
      });
    auto publisher = node->create_publisher<std_msgs::msg::String>(receive_topic, 10);
    auto tools = std::make_shared<vln_agent::ToolManager>(node);
    tools->load(runtime_config.string());
    auto session = std::make_shared<vln_agent::SessionManager>(runtime_config.string());
    session->setTools(tools->getTools());
    vln_agent::UserInteractionManager ui(node, {session, tools, {}});
    ui.load(runtime_config.string());

    const auto discovery_deadline = std::chrono::steady_clock::now() + 10s;
    while (publisher->get_subscription_count() == 0 || subscription->get_publisher_count() == 0) {
      require(std::chrono::steady_clock::now() < discovery_deadline, "ROS 接口发现超时");
      executor.spin_some();
      std::this_thread::sleep_for(20ms);
    }
    const auto nonce = "CTX_" + std::to_string(
      std::chrono::system_clock::now().time_since_epoch().count());
    const Json system{{"role", "system"}, {"content", Json::array({Json{
      {"type", "input_text"},
      {"text", "这是受控软件接口测试。记住首轮用户给出的随机口令。首次回复只调用 "
        "interaction_test_echo 一次。收到工具成功结果后回复随机口令和 TOOL_OK，"
        "不要再次调用工具。后续询问口令时直接回答记住的原始口令。"}}})}};
    session->addInputs(system);
    std_msgs::msg::String input_message;
    input_message.data = Json::array({userInput("随机口令是 " + nonce +
      "。现在必须先调用 interaction_test_echo，不要先回答文字。")}).dump();
    publisher->publish(input_message);
    Json received = Json::array();
    const auto input_deadline = std::chrono::steady_clock::now() + 10s;
    while (received.empty()) {
      require(std::chrono::steady_clock::now() < input_deadline, "ROS 原生用户输入接收超时");
      executor.spin_some();
      received = ui.exchange();
      std::this_thread::sleep_for(20ms);
    }
    require(received == Json::parse(input_message.data), "原生输入被交互组件改写");
    Json latest, displayed_outputs = Json::array(), turns = Json::array();
    std::size_t turn_index = 0;
    auto turn = [&](const Json & inputs) {
      Json sent_request;
      session->addInputs(inputs);
      session->request();
      const auto deadline = std::chrono::steady_clock::now() + 130s;
      while (std::chrono::steady_clock::now() < deadline) {
        const auto request = session->getCurrentRequest();
        if (!request.is_null()) {sent_request = request;}
        executor.spin_some();
        ui.exchange();
        const auto response = session->takeResponse();
        if (!response.is_null()) {
          // 请求体没有认证头，保留原始往返便于定位代理或模型返回异常。
          const auto trace = result_path.parent_path() /
            ("turn_" + std::to_string(++turn_index) + ".json");
          std::ofstream(trace) << Json{{"request", sent_request}, {"response", response}}.dump(2);
          require(!sent_request.is_null(), "未观测到在途请求，无法验证发送内容");
          require(sent_request.at("input") == inputs || turn_index == 1,
            "增量请求混入了未提交的历史输入");
          if (turn_index > 1) {
            require(sent_request.at("previous_response_id") == latest.at("id"),
              "previous_response_id 未续接上一轮响应");
            require(sent_request.at("input").dump().find(nonce) == std::string::npos,
              "后续请求重复发送了初始口令");
          }
          latest = response;
          ui.exchange(latest);
          executor.spin_some();
          return response;
        }
        const auto status = session->getRequestStatus();
        if (status.at("state") == "error") {
          throw std::runtime_error("真实 API 会话错误: " +
            status.at("last_error").at("code").get<std::string>());
        }
        std::this_thread::sleep_for(20ms);
      }
      throw std::runtime_error("真实 API 等待超时");
    };
    const auto first = turn(received);
    turns.push_back(Json{{"response", first}});
    const auto handles = tools->submit(first);
    require(handles.size() == 1, "首轮没有生成唯一测试函数调用");
    require(handles[0].name == "interaction_test_echo", "出现意外工具调用");
    require(handles[0].result.wait_for(5s) == std::future_status::ready, "测试函数未及时结束");
    displayed_outputs.push_back(handles[0].result.get());
    ui.exchange(nullptr, displayed_outputs);
    const auto second = turn(displayed_outputs);
    turns.push_back(Json{{"response", second}, {"tool_outputs", displayed_outputs}});
    require(textOf(second).find(nonce) != std::string::npos, "工具结果续接后丢失初始上下文");
    require(textOf(second).find("TOOL_OK") != std::string::npos, "未确认工具结果");
    const auto third = turn(Json::array({userInput("只回复首轮的随机口令，不调用工具。")}));
    turns.push_back(Json{{"response", third}});
    require(textOf(third).find(nonce) != std::string::npos, "第三轮增量输入丢失上下文");

    // Agent 只交付新增事件；空轮询不应再次产生已经交付的内容。
    for (int i = 0; i < 3; ++i) {
      ui.exchange();
      executor.spin_some();
    }
    std::map<std::string, std::string> displayed_text;
    std::set<std::string> call_ids, output_ids;
    std::size_t calls = 0, outputs = 0;
    for (const auto & packet : packets) {
      if (!packet.contains("response")) {continue;}
      const auto & response = packet.at("response");
      if (response.contains("messages")) {
        for (const auto & part : response.at("messages")) {
          const auto key = part.at("response_id").get<std::string>() + "/" +
            std::to_string(part.at("output_index").get<std::size_t>()) + "/" +
            std::to_string(part.at("content_index").get<std::size_t>());
          if (part.at("op") == "replace") {displayed_text[key] = part.at("text");}
          else {displayed_text[key] += part.at("text").get<std::string>();}
        }
      }
      if (response.contains("function_calls")) {
        for (const auto & call : response.at("function_calls")) {
          ++calls; call_ids.insert(call.at("call_id").get<std::string>());
        }
      }
      if (response.contains("tool_outputs")) {
        for (const auto & output : response.at("tool_outputs")) {
          ++outputs; output_ids.insert(output.at("call_id").get<std::string>());
        }
      }
    }
    require(calls == 1 && call_ids.size() == 1 && outputs == 1 && output_ids.size() == 1,
      "ROS 汇总 JSON 出现重复或缺失的函数调用/执行结果");
    for (const auto & response : {second, third}) {
      std::string shown;
      for (const auto & item : displayed_text) {
        if (item.first.find(response.at("id").get<std::string>() + "/") == 0) {shown += item.second;}
      }
      require(shown == textOf(response), "ROS 展示文本与完整模型回复不一致或存在重复");
    }
    const Json fixture{{"model", "gpt-6-luna"}, {"time_stamp", vln_agent::utils::utcNow()},
      {"nonce", nonce}, {"context_verified", true}, {"turns", turns}};
    std::ofstream(result_path) << fixture.dump(2);
    ui.shutdown();
    session->shutdown();
    tools->shutdown();
    std::cout << Json{{"model", "gpt-6-luna"}, {"turns", turns.size()},
      {"context_verified", true}, {"function_calls", calls}, {"tool_outputs", outputs},
      {"ros_packets", packets.size()}, {"fixture", result_path.string()},
      {"usage", third.value("usage", Json())}}.dump(2) << '\n';
    executor.remove_node(node);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
