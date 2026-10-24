#include <cstdlib>
#include <fstream>
#include <regex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "test_helpers.hpp"
#include "test_interface.hpp"
#include "ws_fixture.hpp"
#include "vln_agent/interaction/user_interaction_manager.hpp"
#include "vln_agent/session/session_manager.hpp"
#include "vln_agent/utils/time.hpp"

namespace vln_agent::test
{
class InteractionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    interfaceTraces().clear();
    node = std::make_shared<rclcpp::Node>("interaction_test");
  }
  Json channel(const std::string & name, const Json & transmit, bool images = false)
  {
    return {{"id", name}, {"plugin", "vln_agent_test/RecordingInterface"},
      {"transmit", transmit}, {"include_images", images}, {"settings", {{"name", name}}}};
  }
  std::string config(const Json & channels)
  {
    return files.write("interaction.yaml",
      Json{{"user_interaction", {{"channels", channels}}}}.dump(2));
  }
  Json model(const std::string & id, const std::string & text)
  {
    auto result = completed(id, text);
    result["output"].push_back(call("echo", id + "_call"));
    return result;
  }
  std::shared_ptr<rclcpp::Node> node;
  FixtureFiles files;
};

TEST_F(InteractionTest, SharesImmutableBatchAndForwardsNativeInputsWithoutWrapping)
{
  const Json transmit{{"response", {{"messages", true}, {"function_calls", true}}}};
  UserInteractionManager ui(node, {});
  ui.load(config(Json::array({channel("a", transmit), channel("b", transmit)})));
  const auto input = message(R"({"source":"user","time_stamp":"2026-10-21T06:00:00.000Z"})");
  interfaceTraces()["a"].inputs.push_back(input);
  interfaceTraces()["b"].inputs.push_back(input);
  const auto response = model("r1", "你好");
  EXPECT_EQ(ui.exchange(response), Json::array({input, input}));
  const auto & a = interfaceTraces().at("a").packets;
  const auto & b = interfaceTraces().at("b").packets;
  ASSERT_EQ(a.size(), 1u);
  ASSERT_EQ(b.size(), 1u);
  EXPECT_EQ(a[0].get(), b[0].get());
  EXPECT_EQ(a[0]->at("response").at("function_calls").size(), 1u);
  EXPECT_TRUE(std::regex_match(a[0]->at("time_stamp").get<std::string>(),
    std::regex(R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z)")));
  EXPECT_TRUE(ui.exchange().empty());
  EXPECT_EQ(a.size(), 1u);
  EXPECT_EQ(b.size(), 1u);
  EXPECT_EQ(interfaceTraces().at("a").polls, 2);
  // 组件不保存已完成 ID；是否投递旧事件由 Agent 决定。
  ui.exchange(response);
  EXPECT_EQ(a.size(), 2u);
  EXPECT_EQ(a[1]->at("response").at("messages")[0].at("text"), "你好");
}

TEST_F(InteractionTest, AppliesPerChannelSelectionAndImageFilteringBeforeDelivery)
{
  const Json transmit{{"response", {{"tool_outputs", true}}}};
  UserInteractionManager ui(node, {});
  ui.load(config(Json::array({channel("hidden", transmit), channel("shown", transmit, true),
    channel("disabled", Json::object())})));
  const Json image{{"type", "input_image"}, {"image_url", "data:image/jpeg;base64,AA=="}};
  const auto result = utils::functionCallOutput("c1",
    Json{{"ok", true}, {"time_stamp", utils::utcNow()}}, {image});
  ui.exchange(nullptr, Json::array({result}));
  const auto & hidden = *interfaceTraces().at("hidden").packets.at(0);
  const auto & shown = *interfaceTraces().at("shown").packets.at(0);
  EXPECT_TRUE(hidden.at("response").at("tool_outputs")[0].at("output")[1].at("omitted"));
  EXPECT_EQ(shown.at("response").at("tool_outputs")[0].at("output")[1], image);
  EXPECT_FALSE(hidden.contains("request"));
  EXPECT_TRUE(interfaceTraces().at("disabled").packets.empty());
  EXPECT_EQ(result.at("output")[1], image);
  std::weak_ptr<const Json> delivered = interfaceTraces().at("shown").packets[0];
  interfaceTraces().at("shown").packets.clear();
  EXPECT_TRUE(delivered.expired());
  ui.exchange();
  EXPECT_EQ(interfaceTraces().at("hidden").packets.size(), 1u);
}

TEST_F(InteractionTest, CommitsOnlyAfterAllChannelsAcceptAndDoesNotResendToEarlierChannel)
{
  const Json transmit{{"response", {{"messages", true}}}};
  UserInteractionManager ui(node, {});
  ui.load(config(Json::array({channel("a", transmit), channel("b", transmit),
    channel("c", transmit)})));
  interfaceTraces()["b"].fail_next = true;
  interfaceTraces()["a"].inputs.push_back(message("buffered"));
  const auto response = model("r_retry", "只发一次");
  EXPECT_THROW(ui.exchange(response), std::runtime_error);
  EXPECT_EQ(interfaceTraces().at("a").packets.size(), 1u);
  EXPECT_TRUE(interfaceTraces().at("c").packets.empty());
  EXPECT_EQ(ui.exchange(response), Json::array({message("buffered")}));
  EXPECT_EQ(interfaceTraces().at("a").packets.size(), 1u);
  const auto first = interfaceTraces().at("a").packets[0];
  EXPECT_EQ(first.get(), interfaceTraces().at("b").packets[0].get());
  EXPECT_EQ(first.get(), interfaceTraces().at("c").packets[0].get());
  ui.exchange();
  EXPECT_EQ(interfaceTraces().at("c").packets.size(), 1u);
}

TEST_F(InteractionTest, StreamAndFinalTextUseOnePositionButFinalOnlyChannelReceivesWholeText)
{
  const auto final = completed("stream1", "你好世界");
  WsFixture server([&](auto & peer, const Json &, std::size_t) {
    peer.send(Json{{"type", "response.created"}, {"response", {{"id", "stream1"}}}});
    peer.send(Json{{"type", "response.output_text.delta"}, {"output_index", 1},
      {"content_index", 0}, {"delta", "你好"}});
    server.waitRelease(0);
    peer.send(Json{{"type", "response.completed"}, {"response", final}});
  });
  auto session = std::make_shared<SessionManager>(
    files.sessionConfig(server.url(), Json{{"stream_output", true}}));
  UserInteractionManager ui(node, {session, {}, {}});
  ui.load(config(Json::array({
    channel("live", Json{{"response", {{"messages", true}, {"stream", true}}}}),
    channel("final", Json{{"response", {{"messages", true}}}})})));
  session->addInputs(message("start"));
  session->request();
  waitFor([&] {return !session->getStreamSnapshot().at("content").empty();});
  ui.exchange();
  EXPECT_EQ(interfaceTraces().at("live").packets[0]->at("response").at("messages")[0].at("text"),
    "你好");
  EXPECT_TRUE(interfaceTraces().at("final").packets.empty());
  ui.exchange();
  EXPECT_EQ(interfaceTraces().at("live").packets.size(), 1u);
  server.release(0);
  waitFor([&] {return session->getRequestStatus().at("response_ready").get<bool>();});
  const auto consumed = session->takeResponse();
  EXPECT_EQ(consumed, final);
  ui.exchange(consumed);
  const auto & live = interfaceTraces().at("live").packets;
  ASSERT_EQ(live.size(), 2u);
  EXPECT_EQ(live[1]->at("response").at("messages")[0].at("text"), "世界");
  EXPECT_EQ(interfaceTraces().at("final").packets[0]->at("response").at("messages")[0].at("text"),
    "你好世界");
  // 响应已交给 Agent，会话和交互层均不再保留它的流式正文。
  EXPECT_TRUE(session->getStreamSnapshot().at("content").empty());
  ui.exchange();
  EXPECT_EQ(live.size(), 2u);
  session->shutdown();
}

TEST_F(InteractionTest, QueriesCurrentRequestOnlyAndPreservesResponseSlotAndToolDefinitions)
{
  WsFixture server([](auto & peer, const Json &, std::size_t) {
    peer.send(Json{{"type", "response.created"}, {"response", {{"id", "inflight"}}}});
  });
  auto session = std::make_shared<SessionManager>(files.sessionConfig(server.url()));
  auto tools = std::make_shared<ToolManager>(node);
  ConfigFile tool_config;
  tools->load(tool_config.config(Json::array()));
  session->setTools(Json::array({definition("session-only")}));
  Json input = message("sent");
  input["content"].push_back(Json{{"type", "input_image"}, {"file_id", "private-image"}});
  session->addInputs(input);
  session->request();
  waitFor([&] {return session->getRequestStatus().at("current_response_id") == "inflight";});
  session->addInputs(message("pending-secret"));
  UserInteractionManager ui(node, {session, tools, {}});
  ui.load(config(Json::array({channel("status", Json{
    {"request", true}, {"tools", {{"definitions", true}, {"status", true}, {"queue", true}}},
    {"session", {{"request_status", true}, {"context_status", true}}}})})));
  ui.exchange();
  const auto & packet = *interfaceTraces().at("status").packets[0];
  EXPECT_FALSE(packet.at("request").contains("tools"));
  EXPECT_TRUE(packet.at("tools").at("definitions").empty());
  EXPECT_EQ(packet.at("session").at("request_status").at("pending_input_count"), 1);
  EXPECT_EQ(packet.dump().find("pending-secret"), std::string::npos);
  EXPECT_EQ(packet.dump().find("private-image"), std::string::npos);
  EXPECT_EQ(session->getCurrentRequest().at("tools")[0].at("name"), "session-only");
  EXPECT_EQ(session->getPendingInputs().size(), 1u);
  EXPECT_TRUE(session->takeResponse().is_null());
}

TEST_F(InteractionTest, CompletedToolStatusAndOutputDoNotConsumeSharedFuture)
{
  auto tools = std::make_shared<ToolManager>(node);
  ConfigFile tool_config;
  tool_config.write("tool.json", Json::array({definition("echo")}).dump());
  const Json function{{"name", "echo"}, {"plugin", "vln_agent_test/TestTool"},
    {"blocking", false}};
  const Json group{{"id", "test"}, {"definition", "tool.json"},
    {"functions", Json::array({function})}};
  tools->load(tool_config.config(Json::array({group})));
  UserInteractionManager ui(node, {{}, tools, {}});
  ui.load(config(Json::array({channel("tools", Json{
    {"response", {{"function_calls", true}, {"tool_outputs", true}}},
    {"tools", {{"definitions", true}, {"queue", true}, {"finished_calls", true}}}})})));
  auto response = model("r_tools", "");
  const auto handles = tools->submit(response);
  ASSERT_EQ(handles.size(), 1u);
  EXPECT_TRUE(payload(handles[0]).at("ok"));
  const auto output = handles[0].result.get();
  ui.exchange(response, Json::array({output}));
  const auto & packet = *interfaceTraces().at("tools").packets.at(0);
  EXPECT_EQ(packet.at("tools").at("finished_calls")[0].at("state"), "finished");
  EXPECT_EQ(packet.at("response").at("tool_outputs")[0], output);
  EXPECT_TRUE(Json::parse(output.at("output")[0].at("text").get<std::string>()).contains("time_stamp"));
  EXPECT_EQ(handles[0].result.get(), output);
  EXPECT_TRUE(tools->getCallStatus(handles[0].call_id).is_null());
  ui.exchange();
  EXPECT_EQ(interfaceTraces().at("tools").packets.size(), 1u);
}

TEST_F(InteractionTest, RollbackClosesInitializedPluginsAndExpiredDependenciesAreReported)
{
  UserInteractionManager ui(node, {});
  auto bad = channel("bad", Json::object());
  bad["settings"]["fail_initialize"] = true;
  EXPECT_THROW(ui.load(config(Json::array({bad}))), std::runtime_error);
  EXPECT_EQ(interfaceTraces().at("bad").shutdowns, 1);
  ui.load(config(Json::array({channel("needs-session", Json{{"request", true}})})));
  EXPECT_THROW(ui.exchange(), std::logic_error);
  ui.shutdown();
  ui.shutdown();
  EXPECT_THROW(ui.exchange(), std::logic_error);
}

TEST_F(InteractionTest, RosTopicPluginPublishesOneJsonAndReturnsReceivedNativeArray)
{
  const auto send_topic = "/interaction_test/output";
  const auto receive_topic = "/interaction_test/input";
  Json received;
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    send_topic, 10, [&](std_msgs::msg::String::ConstSharedPtr msg) {
      received = Json::parse(msg->data);
    });
  auto publisher = node->create_publisher<std_msgs::msg::String>(receive_topic, 10);
  UserInteractionManager ui(node, {});
  ui.load(config(Json::array({Json{
    {"id", "ros"}, {"plugin", "vln_agent/RosTopicInterface"},
    {"transmit", {{"response", {{"messages", true}}}}},
    {"settings", {{"send_topic", send_topic}, {"receive_topic", receive_topic}}}}})));
  ASSERT_TRUE(eventually([&] {
    rclcpp::spin_some(node);
    return publisher->get_subscription_count() > 0 && subscription->get_publisher_count() > 0;
  }));
  ui.exchange(model("ros1", "已接收"));
  ASSERT_TRUE(eventually([&] {rclcpp::spin_some(node); return !received.is_null();}));
  EXPECT_EQ(received.at("response").at("messages")[0].at("text"), "已接收");
  const auto inputs = Json::array({message(
    R"({"source":"user","time_stamp":"2026-10-21T06:00:00.000Z","instruction":"检查任务"})")});
  std_msgs::msg::String outgoing;
  outgoing.data = inputs.dump();
  publisher->publish(outgoing);
  Json actual = Json::array();
  ASSERT_TRUE(eventually([&] {
    rclcpp::spin_some(node);
    actual = ui.exchange();
    return !actual.empty();
  }));
  EXPECT_EQ(actual, inputs);
  EXPECT_TRUE(ui.exchange().empty());
}

TEST_F(InteractionTest, ReplaysRecordedApiResponsesWithoutNetwork)
{
  const auto * path = std::getenv("VLN_LIVE_FIXTURE");
  if (!path) {GTEST_SKIP() << "显式运行 live_interaction_test 生成真实响应后设置 VLN_LIVE_FIXTURE";}
  std::ifstream input(path);
  ASSERT_TRUE(input.good());
  const auto fixture = Json::parse(input);
  ASSERT_EQ(fixture.at("model"), "gpt-6-luna");
  UserInteractionManager ui(node, {});
  ui.load(config(Json::array({channel("live-api", Json{{"response",
    {{"messages", true}, {"function_calls", true}, {"tool_outputs", true}}}})})));
  for (const auto & turn : fixture.at("turns")) {
    ui.exchange(turn.at("response"), turn.value("tool_outputs", Json::array()));
    const auto count = interfaceTraces().at("live-api").packets.size();
    ui.exchange();
    EXPECT_EQ(interfaceTraces().at("live-api").packets.size(), count);
  }
  std::string text;
  std::size_t calls = 0, outputs = 0;
  for (const auto & packet : interfaceTraces().at("live-api").packets) {
    const auto & response = packet->at("response");
    if (response.contains("messages")) {
      for (const auto & message : response.at("messages")) {text += message.at("text").get<std::string>();}
    }
    if (response.contains("function_calls")) {calls += response.at("function_calls").size();}
    if (response.contains("tool_outputs")) {outputs += response.at("tool_outputs").size();}
  }
  EXPECT_GE(calls, 1u);
  EXPECT_EQ(outputs, 1u);
  EXPECT_NE(text.find(fixture.at("nonce").get<std::string>()), std::string::npos);
  EXPECT_TRUE(fixture.at("context_verified").get<bool>());
}
}  // namespace vln_agent::test
