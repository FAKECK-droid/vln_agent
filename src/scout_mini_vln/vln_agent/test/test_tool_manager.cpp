#include "test_helpers.hpp"
#include "test_tool.hpp"

#include <algorithm>
#include <stdexcept>

namespace vln_agent::test
{

/** @brief 仅注入普通节点，核心队列测试不需要 ROS 回调。 */
class ToolManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("tool_manager_test");
    std::lock_guard<std::mutex> lock(probe().mutex);
    probe().started.clear();
    probe().released.clear();
    probe().active = 0;
    probe().peak = 0;
    probe().resource_shutdowns = 0;
    config.write("tools.json", Json::array({
          definition("first"), definition("second"), definition("third"), definition("unused")}).dump());
  }

  Json function(const std::string & name, bool blocking = false)
  {
    return {{"name", name}, {"plugin", "vln_agent_test/TestTool"}, {"blocking", blocking}};
  }

  Json group(const Json & functions, const std::string & id = "test")
  {
    return {{"id", id}, {"definition", "tools.json"},
      {"settings", {{"number", 7}, {"quoted", "007"}, {"enabled", true}}},
      {"functions", functions}};
  }

  void started(std::size_t count)
  {
    std::unique_lock<std::mutex> lock(probe().mutex);
    ASSERT_TRUE(probe().condition.wait_for(lock, 3s, [&] {
      return probe().started.size() >= count;
    }));
  }

  void release(const std::string & id)
  {
    std::lock_guard<std::mutex> lock(probe().mutex);
    probe().released.insert(id);
    probe().condition.notify_all();
  }

  ConfigFile config;
  rclcpp::Node::SharedPtr node;
};

TEST_F(ToolManagerTest, SelectsDefinitionsInConfigurationOrderAndInjectsSettings)
{
  auto second = function("second");
  second["settings"] = {{"label", "002"}, {"ratio", 0.5}};
  ToolManager manager(node);
  manager.load(config.config(Json::array({group(Json::array({second, function("first")}))})));
  EXPECT_EQ(manager.getTools(), Json::array({definition("second"), definition("first")}));
  auto handles = manager.submit(response(Json::array({
        call("first", "a"), call("second", "b")})));
  const auto first_result = payload(handles[0]);
  const auto second_result = payload(handles[1]);
  EXPECT_EQ(first_result.at("token_id"), second_result.at("token_id"));
  EXPECT_EQ(first_result.at("settings"), Json::object());
  EXPECT_EQ(second_result.at("settings"), second.at("settings"));
  EXPECT_EQ(second_result.at("group_settings"),
    (Json{{"number", 7}, {"quoted", "007"}, {"enabled", true}}));
}

TEST_F(ToolManagerTest, SeparatesGroupsAndManagerInstances)
{
  ToolManager first(node), second(node);
  const auto path = config.config(Json::array({
        group(Json::array({function("first")}), "one"),
        group(Json::array({function("second")}), "two")}));
  first.load(path);
  second.load(path);
  const auto a = first.submit(response(Json::array({call("first", "a"), call("second", "b")})));
  const auto b = second.submit(response(Json::array({call("first", "a")})));
  EXPECT_NE(payload(a[0]).at("token_id"), payload(a[1]).at("token_id"));
  EXPECT_NE(payload(a[0]).at("token_id"), payload(b[0]).at("token_id"));
}

TEST_F(ToolManagerTest, PublishesFastResultIndependentlyAndDeduplicatesCallIds)
{
  ToolManager manager(node);
  manager.load(config.config(Json::array({group(Json::array({function("first")}))})));
  const auto handles = manager.submit(response(Json::array({
        call("first", "slow", {{"id", "slow"}, {"wait", true}}),
        call("first", "fast", {{"id", "fast"}})})));
  started(2);
  EXPECT_EQ(payload(handles[1]).at("id"), "fast");
  EXPECT_EQ(handles[0].result.wait_for(0ms), std::future_status::timeout);
  auto repeated = call("unknown", "slow");
  repeated["arguments"] = "invalid";
  const auto duplicate = manager.submit(response(Json::array({repeated})));
  EXPECT_EQ(duplicate[0].name, "first");
  EXPECT_EQ(duplicate[0].result.wait_for(0ms), std::future_status::timeout);
  release("slow");
  EXPECT_EQ(payload(handles[0]), payload(duplicate[0]));
  std::lock_guard<std::mutex> lock(probe().mutex);
  EXPECT_EQ(probe().started.size(), 2u);
}

TEST_F(ToolManagerTest, BlockingCallPausesLaterDispatchWithoutWaitingForEarlierCalls)
{
  ToolManager manager(node);
  manager.load(config.config(Json::array({group(Json::array({
          function("first"), function("second", true), function("third")}))}), 2));
  const auto handles = manager.submit(response(Json::array({
        call("first", "a", {{"id", "a"}, {"wait", true}}),
        call("second", "b", {{"id", "b"}, {"wait", true}}),
        call("third", "c", {{"id", "c"}})})));
  started(2);
  {
    std::lock_guard<std::mutex> lock(probe().mutex);
    EXPECT_EQ(probe().started.size(), 2u);
    EXPECT_EQ(std::count(probe().started.begin(), probe().started.end(), "c"), 0);
  }
  release("b");
  EXPECT_TRUE(payload(handles[1]).at("ok"));
  EXPECT_EQ(payload(handles[2]).at("id"), "c");
  EXPECT_EQ(handles[0].result.wait_for(0ms), std::future_status::timeout);
  release("a");
  EXPECT_TRUE(payload(handles[0]).at("ok"));
}

TEST_F(ToolManagerTest, BoundsActiveExecutions)
{
  ToolManager manager(node);
  manager.load(config.config(Json::array({group(Json::array({function("first")}))}), 2));
  Json items = Json::array();
  for (int i = 0; i < 5; ++i) {
    const auto id = std::to_string(i);
    items.push_back(call("first", id, {{"id", id}, {"wait", true}}));
  }
  const auto handles = manager.submit(response(items));
  started(2);
  {
    std::lock_guard<std::mutex> lock(probe().mutex);
    EXPECT_EQ(probe().started.size(), 2u);
    EXPECT_EQ(probe().peak, 2);
  }
  release("0");
  started(3);
  for (int i = 1; i < 5; ++i) {
    release(std::to_string(i));
  }
  for (const auto & handle : handles) {
    EXPECT_TRUE(payload(handle).at("ok"));
  }
  EXPECT_LE(probe().peak, 2);
}

TEST_F(ToolManagerTest, LimitsPendingQueueAndReturnsCorrelatedFailure)
{
  ToolManager manager(node);
  manager.load(config.config(Json::array({group(Json::array({function("first")}))}), 1, 1));
  const auto handles = manager.submit(response(Json::array({
        call("first", "a", {{"id", "a"}, {"wait", true}}),
        call("first", "b"), call("first", "c")})));
  EXPECT_EQ(payload(handles[1]).at("code"), "TOOL_QUEUE_FULL");
  EXPECT_EQ(payload(handles[2]).at("code"), "TOOL_QUEUE_FULL");
  release("a");
  EXPECT_TRUE(payload(handles[0]).at("ok"));
}

TEST_F(ToolManagerTest, TurnsCallErrorsIntoIndividualResultsAndKeepsImagesNative)
{
  ToolManager manager(node);
  manager.load(config.config(Json::array({group(Json::array({function("first")}))})));
  auto malformed = call("first", "bad_json");
  malformed["arguments"] = "{";
  auto unfinished = call("first", "unfinished");
  unfinished["status"] = "in_progress";
  const Json image = {{"type", "input_image"}, {"image_url", "data:image/jpeg;base64,AA=="}};
  const auto handles = manager.submit(response(Json::array({
        call("unknown", "unknown"), malformed, call("first", "non_object", Json::array()),
        call("first", "exception", {{"throw", true}}), unfinished,
        call("first", "image", {{"images", Json::array({image})}})})));
  EXPECT_EQ(payload(handles[0]).at("code"), "UNKNOWN_TOOL");
  EXPECT_EQ(payload(handles[1]).at("code"), "INVALID_ARGUMENTS");
  EXPECT_EQ(payload(handles[2]).at("code"), "INVALID_ARGUMENTS");
  EXPECT_EQ(payload(handles[3]).at("code"), "TOOL_EXECUTION_FAILED");
  EXPECT_EQ(payload(handles[4]).at("code"), "INVALID_CALL");
  EXPECT_TRUE(payload(handles[5]).at("ok"));
  const auto output = handles[5].result.get();
  EXPECT_EQ(output.at("output").size(), 2u);
  EXPECT_EQ(output.at("output").at(1), image);
}

TEST_F(ToolManagerTest, ShutdownCompletesRunningAndQueuedPromisesAndRetainsDeduplication)
{
  ToolManager manager(node);
  manager.load(config.config(Json::array({group(Json::array({function("first")}))}), 1));
  const auto handles = manager.submit(response(Json::array({
        call("first", "running", {{"id", "running"}, {"wait", true}}),
        call("first", "pending")})));
  started(1);
  manager.shutdown();
  manager.shutdown();
  EXPECT_EQ(payload(handles[0]).at("code"), "TOOL_MANAGER_CLOSED");
  EXPECT_EQ(payload(handles[1]).at("code"), "TOOL_MANAGER_CLOSED");
  const auto after = manager.submit(response(Json::array({
        call("first", "new"), call("first", "pending")})));
  EXPECT_EQ(payload(after[0]).at("code"), "TOOL_MANAGER_CLOSED");
  EXPECT_EQ(payload(after[1]), payload(handles[1]));
  EXPECT_TRUE(manager.getTools().empty());
  EXPECT_EQ(probe().resource_shutdowns, 1);
}

TEST_F(ToolManagerTest, IgnoresMessagesAndContextItemsAndRejectsIncompleteResponses)
{
  ToolManager manager(node);
  manager.load(config.config(Json::array()));
  EXPECT_TRUE(manager.submit(response(Json::array({
          {{"type", "message"}, {"content", Json::array()}},
          {{"type", "reasoning"}, {"encrypted_content", "opaque"}},
          {{"type", "compaction"}, {"encrypted_content", "opaque"}}}))).empty());
  EXPECT_THROW(manager.submit({{"status", "in_progress"}, {"output", Json::array()}}),
    std::invalid_argument);
}

TEST_F(ToolManagerTest, LoadingFailureIsAtomicAndCanBeRetriedBeforeSuccessfulLoad)
{
  auto broken = function("second");
  broken["settings"] = {{"fail_initialize", true}};
  ToolManager manager(node);
  const auto bad = config.config(Json::array({
        group(Json::array({function("first"), broken}))}));
  EXPECT_THROW(manager.load(bad), std::runtime_error);
  EXPECT_TRUE(manager.getTools().empty());
  EXPECT_EQ(probe().resource_shutdowns, 1);
  manager.load(config.config(Json::array({group(Json::array({function("first")}))})));
  EXPECT_EQ(manager.getTools().size(), 1u);
  EXPECT_THROW(manager.load(bad), std::logic_error);
}

TEST_F(ToolManagerTest, RejectsBasicConfigurationErrors)
{
  const auto valid_group = group(Json::array({function("first")}));
  const std::vector<Json> invalid_groups{
    Json::array({valid_group, valid_group}),
    Json::array({group(Json::array({function("first"), function("first")}))}),
    Json::array({group(Json::array({function("missing")}))})};
  for (const auto & groups : invalid_groups) {
    ToolManager manager(node);
    EXPECT_THROW(manager.load(config.config(groups)), std::exception);
    EXPECT_TRUE(manager.getTools().empty());
  }
  auto absent_plugin = function("first");
  absent_plugin["plugin"] = "missing/Plugin";
  ToolManager missing(node);
  EXPECT_THROW(missing.load(config.config(Json::array({group(Json::array({absent_plugin}))}))),
    std::exception);
  ToolManager zero(node);
  EXPECT_THROW(zero.load(config.config(Json::array(), 0)), std::invalid_argument);
  config.write("tools.json", "{}");
  ToolManager object(node);
  EXPECT_THROW(object.load(config.config(Json::array({valid_group}))), std::invalid_argument);
}

/** @brief 注册表测试资源，使用外部计数验证关闭只执行一次。 */
class Resource : public SharedResource
{
public:
  explicit Resource(int & count) : count_(count) {}
  void shutdown() noexcept override {++count_;}
private:
  int & count_;
};

class OtherResource : public Resource
{
public:
  using Resource::Resource;
};

TEST(SharedResourcesTest, CreatesOnceChecksTypesAndClosesOnce)
{
  int created = 0;
  int closed = 0;
  SharedResources resources;
  const auto factory = [&] {
      ++created;
      return std::make_shared<Resource>(closed);
    };
  const auto first = resources.getOrCreate<Resource>("one", factory);
  EXPECT_EQ(first, resources.getOrCreate<Resource>("one", factory));
  EXPECT_EQ(created, 1);
  EXPECT_THROW(resources.getOrCreate<OtherResource>("one", [&] {
        return std::make_shared<OtherResource>(closed);
      }), std::invalid_argument);
  resources.shutdown();
  resources.shutdown();
  EXPECT_EQ(closed, 1);
  EXPECT_THROW(resources.getOrCreate<Resource>("new", factory), std::logic_error);
}

}  // namespace vln_agent::test
