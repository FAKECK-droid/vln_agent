#include "vln_agent/vln_agent.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "vln_agent/utils/time.hpp"

namespace vln_agent
{
VLNAgent::VLNAgent(const rclcpp::NodeOptions & options)
: Node("vln_agent", options)
{
}

VLNAgent::~VLNAgent()
{
  shutdown();
}

void VLNAgent::init()
{
  const auto config_path = declare_parameter<std::string>("global_yaml_path",
    ament_index_cpp::get_package_share_directory("vln_agent") + "/config/agent.yaml");
  const auto prompt_path = declare_parameter<std::string>("system_prompt_path", "");
  const auto instruction = declare_parameter<std::string>("initial_instruction", "");
  loop_hz_ = declare_parameter<double>("loop_hz", 20.0);
  if (!std::isfinite(loop_hz_) || loop_hz_ <= 0) {
    throw std::invalid_argument("loop_hz 必须为正数");
  }

  if (prompt_path.empty()) {
    throw std::invalid_argument("请通过 system_prompt_path 指定系统提示词文件");
  }
  std::ifstream file(prompt_path);
  if (!file) {throw std::runtime_error("无法读取系统提示词: " + prompt_path);}
  const std::string prompt{
    std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};

  auto node = shared_from_this();
  event_manager_ = std::make_shared<EventManager>(node);
  event_manager_->load(config_path);
  tool_manager_ = std::make_shared<ToolManager>(node);
  tool_manager_->load(config_path);
  session_manager_ = std::make_shared<SessionManager>(config_path);
  session_manager_->setTools(tool_manager_->getTools());
  user_interaction_manager_ = std::make_shared<UserInteractionManager>(
    node, UserInteractionSources{session_manager_, tool_manager_, {}});
  user_interaction_manager_->load(config_path);

  Json inputs = Json::array({{{"role", "system"}, {"content", Json::array({
    {{"type", "input_text"}, {"text", prompt}}})}}});
  if (!instruction.empty()) {
    const Json payload{{"source", "user"}, {"time_stamp", utils::utcNow()},
      {"instruction", instruction}};
    inputs.push_back({{"role", "user"}, {"content", Json::array({
      {{"type", "input_text"}, {"text", payload.dump()}}})}});
  }
  addInputs(inputs);
  session_manager_->request();
  RCLCPP_INFO(get_logger(), "Agent 已初始化，主循环频率 %.1f Hz", loop_hz_);
}

void VLNAgent::addInputs(const Json & inputs)
{
  if (inputs.empty()) {return;}
  session_manager_->addInputs(inputs);
  history_.push_back({{"time_stamp", utils::utcNow()}, {"input", inputs}});
}

void VLNAgent::step()
{
  // 取走本轮机器事件，原样追加到会话；在途请求期间由会话保留为待发输入。
  addInputs(event_manager_->takeEvents());

  // 取走模型响应
  outgoing_response_ = session_manager_->takeResponse();

  // 交付模型工具调用
  if (!outgoing_response_.is_null()) {
    auto calls = tool_manager_->submit(outgoing_response_);
    active_calls_.insert(active_calls_.end(), calls.begin(), calls.end());
  }

  // 检查工具执行结果，并将输出追加到下一轮输入
  for (auto it = active_calls_.begin(); it != active_calls_.end();) {
    if (it->result.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      ++it;
      continue;
    }
    const auto & output = it->result.get();
    addInputs(output);
    outgoing_tool_outputs_.push_back(output);
    it = active_calls_.erase(it);
  }

  // 交付用户接口输入，追加到下一轮输入
  auto inputs = user_interaction_manager_->exchange(outgoing_response_, outgoing_tool_outputs_);
  outgoing_response_ = nullptr;
  outgoing_tool_outputs_ = Json::array();
  addInputs(inputs);

  // 只登记有新输入的正常请求；限频由 SessionManager 完成，不自动重放失败请求。
  const auto status = session_manager_->getRequestStatus();
  if (status.at("pending_input_count").get<std::size_t>() > 0 &&
    !status.at("retry_required").get<bool>())
  {
    session_manager_->request();
  }
}

void VLNAgent::run()
{
  rclcpp::ExecutorOptions options;
  options.context = get_node_base_interface()->get_context();
  rclcpp::executors::SingleThreadedExecutor executor(options);
  executor.add_node(shared_from_this());
  rclcpp::WallRate rate(loop_hz_);
  while (rclcpp::ok(options.context)) {
    executor.spin_some();
    if (!rclcpp::ok(options.context)) {break;}
    try {
      step();
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Agent 主循环本轮处理失败: %s", error.what());
    }
    rate.sleep();
  }
  shutdown();
  executor.remove_node(get_node_base_interface());
}

void VLNAgent::shutdown()
{
  if (event_manager_) {event_manager_->shutdown();}
  if (user_interaction_manager_) {user_interaction_manager_->shutdown();}
  if (session_manager_) {session_manager_->shutdown();}
  if (tool_manager_) {tool_manager_->shutdown();}
  active_calls_.clear();
  outgoing_response_ = nullptr;
  outgoing_tool_outputs_ = Json::array();
}
}  // namespace vln_agent

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto agent = std::make_shared<vln_agent::VLNAgent>();
    agent->init();
    agent->run();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("vln_agent"), "Agent 退出: %s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
