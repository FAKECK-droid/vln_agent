#include "test_helpers.hpp"
#include "test_tool.hpp"
#include "vln_agent/srv/tool_call.hpp"

#include <atomic>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

namespace vln_agent::test
{

/** @brief 独立 ROS 客户端验证服务入口；工具仍使用测试插件，不连接机器人。 */
class ToolCallServiceTest : public ::testing::Test
{
protected:
  using Service = srv::ToolCall;
  using Future = rclcpp::Client<Service>::FutureAndRequestId;

  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    server = std::make_shared<rclcpp::Node>("tool_call_server_test");
    client_node = std::make_shared<rclcpp::Node>("tool_call_client_test");
    manager = std::make_unique<ToolManager>(server);
    service_name = "/vln/test/tool_call_" + std::to_string(getpid());
    client = client_node->create_client<Service>(service_name);
    config.write("tools.json", Json::array({definition("first"), definition("second")}).dump());
    {
      std::lock_guard<std::mutex> lock(probe().mutex);
      probe().started.clear(); probe().released.clear();
      probe().active = 0; probe().peak = 0; probe().resource_shutdowns = 0;
    }
    executor.add_node(server);
    executor.add_node(client_node);
    spinner = std::thread([this] {executor.spin();});
  }

  void TearDown() override
  {
    manager->shutdown();
    executor.cancel();
    spinner.join();
  }

  void load(const std::string & name = "", std::size_t concurrent = 2)
  {
    const auto group = Json{{"id", "test"}, {"definition", "tools.json"},
      {"functions", Json::array({
          {{"name", "first"}, {"plugin", "vln_agent_test/TestTool"}, {"blocking", false}},
          {{"name", "second"}, {"plugin", "vln_agent_test/TestTool"}, {"blocking", false}}})}};
    const auto path = config.config(Json::array({group}), concurrent);
    std::ifstream file(path);
    auto settings = Json::parse(file);
    settings["tool_manager"]["service_name"] = name.empty() ? service_name : name;
    config.write("agent.yaml", settings.dump());
    manager->load(path);
    ASSERT_TRUE(client->wait_for_service(3s));
  }

  Future send(const Json & response)
  {
    auto request = std::make_shared<Service::Request>();
    request->request_json = response.dump();
    return client->async_send_request(request);
  }

  Service::Response::SharedPtr receive(Future & future)
  {
    if (future.wait_for(3s) != std::future_status::ready) {
      throw std::runtime_error("ToolCall 服务未在期限内返回");
    }
    return future.get();
  }

  void started(std::size_t count)
  {
    std::unique_lock<std::mutex> lock(probe().mutex);
    ASSERT_TRUE(probe().condition.wait_for(lock, 3s,
      [&] {return probe().started.size() >= count;}));
  }

  ConfigFile config;
  rclcpp::Node::SharedPtr server, client_node;
  rclcpp::Client<Service>::SharedPtr client;
  std::unique_ptr<ToolManager> manager;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spinner;
  std::string service_name;
};

TEST_F(ToolCallServiceTest, ReturnsOrderedNativeOutputsAndUsesTheSameSharedResources)
{
  load();
  const auto direct = manager->submit(response(Json::array({call("first", "model_1")}))).at(0);
  const auto direct_payload = payload(direct);
  const Json image{{"type", "input_image"}, {"image_url", "data:image/png;base64," + std::string(128 * 1024, 'A')},
    {"detail", "auto"}};
  const Json metadata = Json::array({{{"metadata", {{"environment_id", "fixture_1"}}}}});
  auto future = send(response(Json::array({
      call("second", "robot_1", {{"id", "one"}, {"images", Json::array({image})},
        {"image_metadata", metadata}, {"time_stamp", "2026-11-07T06:00:00.000Z"}}),
      call("first", "robot_2", {{"id", "two"}})})));
  const auto reply = receive(future);
  EXPECT_TRUE(reply->error.empty());
  const auto outputs = Json::parse(reply->outputs_json);
  ASSERT_EQ(outputs.size(), 2u);
  EXPECT_EQ(outputs[0].at("type"), "function_call_output");
  EXPECT_EQ(outputs[0].at("call_id"), "robot_1");
  EXPECT_EQ(outputs[1].at("call_id"), "robot_2");
  const auto first = Json::parse(outputs[0].at("output")[0].at("text").get<std::string>());
  const auto second = Json::parse(outputs[1].at("output")[0].at("text").get<std::string>());
  EXPECT_EQ(first.at("data").at("token_id"), direct_payload.at("data").at("token_id"));
  EXPECT_EQ(second.at("data").at("token_id"), direct_payload.at("data").at("token_id"));
  EXPECT_EQ(first.at("images"), metadata);
  EXPECT_EQ(first.at("time_stamp"), "2026-11-07T06:00:00.000Z");
  EXPECT_EQ(outputs[0].at("output")[1], image);
  EXPECT_EQ(manager->getStatus().at("call_count"), 0);
}

TEST_F(ToolCallServiceTest, ReusesActiveHandlesAndDoesNotBlockTheHostExecutor)
{
  load();
  std::atomic_int ticks{0};
  const auto timer = server->create_wall_timer(5ms, [&] {++ticks;});
  const auto direct = manager->submit(response(Json::array({
      call("first", "shared_call", {{"id", "model"}, {"wait", true}})}))).at(0);
  started(1);
  auto future = send(response(Json::array({
      call("unknown", "shared_call"), call("second", "robot_extra", {{"id", "extra"}})})));
  started(2);
  EXPECT_EQ(future.wait_for(0ms), std::future_status::timeout);
  EXPECT_TRUE(eventually([&] {return ticks.load() > 0;}));
  {
    std::lock_guard<std::mutex> lock(probe().mutex);
    probe().released.insert("model");
  }
  probe().condition.notify_all();
  const auto reply = receive(future);
  EXPECT_TRUE(reply->error.empty());
  const auto outputs = Json::parse(reply->outputs_json);
  EXPECT_EQ(outputs[0], direct.result.get());
  EXPECT_EQ(manager->getStatus().at("completed_count"), 2);
  std::lock_guard<std::mutex> lock(probe().mutex);
  EXPECT_EQ(probe().started.size(), 2u);
  EXPECT_EQ(probe().peak, 2);
}

TEST_F(ToolCallServiceTest, SeparatesRequestErrorsFromExistingPerToolErrors)
{
  load();
  auto invalid = std::make_shared<Service::Request>();
  invalid->request_json = "{";
  auto malformed = client->async_send_request(invalid);
  const auto rejected = receive(malformed);
  EXPECT_FALSE(rejected->error.empty());
  EXPECT_EQ(Json::parse(rejected->outputs_json), Json::array());

  auto bad_arguments = call("first", "bad_arguments");
  bad_arguments["arguments"] = "{";
  auto future = send(response(Json::array({call("missing", "unknown"), bad_arguments,
      call("first", "throw", {{"throw", true}})})));
  const auto reply = receive(future);
  EXPECT_TRUE(reply->error.empty());
  const auto outputs = Json::parse(reply->outputs_json);
  ASSERT_EQ(outputs.size(), 3u);
  for (std::size_t i = 0; i < outputs.size(); ++i) {
    const auto body = Json::parse(outputs[i].at("output")[0].at("text").get<std::string>());
    EXPECT_FALSE(body.at("ok").get<bool>());
    EXPECT_EQ(body.at("code"), i == 0 ? "UNKNOWN_TOOL" :
      i == 1 ? "INVALID_ARGUMENTS" : "TOOL_EXECUTION_FAILED");
  }
  auto empty = send(response(Json::array()));
  EXPECT_EQ(Json::parse(receive(empty)->outputs_json), Json::array());
}

TEST_F(ToolCallServiceTest, ShutdownFinishesRunningAndQueuedServiceCalls)
{
  load("", 1);
  auto future = send(response(Json::array({
      call("first", "running", {{"wait", true}}), call("second", "queued")})));
  started(1);
  manager->shutdown();
  const auto reply = receive(future);
  EXPECT_TRUE(reply->error.empty());
  const auto outputs = Json::parse(reply->outputs_json);
  ASSERT_EQ(outputs.size(), 2u);
  for (const auto & output : outputs) {
    const auto body = Json::parse(output.at("output")[0].at("text").get<std::string>());
    EXPECT_EQ(body.at("code"), "TOOL_MANAGER_CLOSED");
  }
  EXPECT_EQ(manager->getStatus().at("call_count"), 0);
  EXPECT_TRUE(eventually([&] {return !client->service_is_ready();}));
}

TEST_F(ToolCallServiceTest, InvalidServiceNameDoesNotCommitPartialLoad)
{
  EXPECT_THROW(load("bad service name"), std::exception);
  EXPECT_TRUE(manager->getTools().empty());
  load();
  auto future = send(response(Json::array({call("first", "after_retry")})));
  EXPECT_TRUE(receive(future)->error.empty());
}
}  // namespace vln_agent::test
