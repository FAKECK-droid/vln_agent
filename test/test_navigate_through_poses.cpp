#include "test_helpers.hpp"

#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace vln_agent::test
{

/** @brief 只连接本地假 Action，不启动 Nav2 或底盘。 */
class NavigateThroughPosesTest : public ::testing::Test
{
protected:
  using Action = nav2_msgs::action::NavigateThroughPoses;
  using Goal = rclcpp_action::ServerGoalHandle<Action>;
  using SingleAction = nav2_msgs::action::NavigateToPose;
  using SingleGoal = rclcpp_action::ServerGoalHandle<SingleAction>;

  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    client = std::make_shared<rclcpp::Node>("through_poses_test_client");
    server_node = std::make_shared<rclcpp::Node>("through_poses_test_server");
    executor.add_node(client);
    executor.add_node(server_node);
    spinner = std::thread([this] {executor.spin();});
    share = ament_index_cpp::get_package_share_directory("vln_agent");
    action_name = "/vln_agent_tests/through_poses_" + std::to_string(getpid());
    single_name = "/vln_agent_tests/single_pose_" + std::to_string(getpid());
  }

  void TearDown() override
  {
    executor.cancel();
    spinner.join();
  }

  void createServer(bool defaults = false)
  {
    server = rclcpp_action::create_server<Action>(
      server_node, defaults ? "/navigate_through_poses" : action_name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        return goal->poses.front().pose.position.x < 0 ?
               rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<Goal>) {
        return reject_cancel ? rclcpp_action::CancelResponse::REJECT : rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<Goal> goal) {
        std::lock_guard<std::mutex> lock(mutex);
        goals.push_back(std::move(goal));
      });
    readiness = rclcpp_action::create_client<Action>(client,
      defaults ? "/navigate_through_poses" : action_name);
    ASSERT_TRUE(readiness->wait_for_action_server(3s));
  }

  void createSingleServer()
  {
    single_server = rclcpp_action::create_server<SingleAction>(
      server_node, single_name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const SingleAction::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<SingleGoal>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<SingleGoal> goal) {
        std::lock_guard<std::mutex> lock(mutex);
        single_goal = std::move(goal);
      });
    single_readiness = rclcpp_action::create_client<SingleAction>(client, single_name);
    ASSERT_TRUE(single_readiness->wait_for_action_server(3s));
  }

  Json group(bool defaults = false)
  {
    Json functions = Json::array();
    for (const auto & prefix : {"navigate_to_pose_", "navigate_through_poses_"}) {
      const std::string cls = std::string(prefix) == "navigate_to_pose_" ?
        "NavigateToPose" : "NavigateThroughPoses";
      for (const auto & item : std::vector<std::pair<std::string, std::string>>{
          {"start_navigation", "StartNavigation"}, {"get_feedback", "GetFeedback"},
          {"cancel_navigation", "CancelNavigation"}})
      {
        functions.push_back({{"name", std::string(prefix) + item.first},
            {"plugin", "vln_agent/" + cls + item.second}, {"blocking", item.first != "get_feedback"}});
      }
    }
    Json settings{{"action_name", single_name}};
    if (!defaults) {settings["navigate_through_poses_action_name"] = action_name;}
    return {{"id", "navigation"}, {"definition", share + "/plugins/tools/navigation/tool.json"},
      {"settings", settings}, {"functions", functions}};
  }

  static Json arguments()
  {
    return {{"frame_id", "map"}, {"poses", Json::array({
        {{"x", 1.0}, {"y", 2.0}, {"yaw", 0.0}},
        {{"x", 3.0}, {"y", 4.0}, {"yaw", M_PI / 2}},
        {{"x", 5.0}, {"y", -2.0}, {"yaw", -M_PI}}})}};
  }

  Json invoke(ToolManager & manager, const std::string & name, const Json & args)
  {
    return payload(manager.submit(response(Json::array({
          call(name, "call_" + std::to_string(++next_call), args)}))).at(0));
  }

  Json start(ToolManager & manager, const Json & args = arguments())
  {
    return invoke(manager, "navigate_through_poses_start_navigation", args);
  }

  Json startReady(ToolManager & manager, const Json & args = arguments())
  {
    Json result;
    if (!eventually([&] {
        result = start(manager, args);
        return result.value("code", "") != "NAVIGATION_UNAVAILABLE";
      })) {throw std::runtime_error("多点 Action 尚未就绪");}
    return result;
  }

  Json feedback(ToolManager & manager, const Json & id)
  {
    return invoke(manager, "navigate_through_poses_get_feedback", {{"task_id", id}});
  }

  std::shared_ptr<Goal> goal(std::size_t index)
  {
    if (!eventually([&] {
        std::lock_guard<std::mutex> lock(mutex);
        return goals.size() > index;
      })) {throw std::runtime_error("假多点 Action 没有收到目标");}
    std::lock_guard<std::mutex> lock(mutex);
    return goals.at(index);
  }

  void expectReleased(ToolManager & manager, const Json & id)
  {
    EXPECT_TRUE(eventually([&] {return feedback(manager, id).value("code", "") == "TASK_NOT_FOUND";}));
  }

  ConfigFile config;
  rclcpp::Node::SharedPtr client, server_node;
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 2};
  std::thread spinner;
  rclcpp_action::Server<Action>::SharedPtr server;
  rclcpp_action::Client<Action>::SharedPtr readiness;
  rclcpp_action::Server<SingleAction>::SharedPtr single_server;
  rclcpp_action::Client<SingleAction>::SharedPtr single_readiness;
  std::mutex mutex;
  std::vector<std::shared_ptr<Goal>> goals;
  std::shared_ptr<SingleGoal> single_goal;
  std::atomic_bool reject_cancel{false};
  std::string share, action_name, single_name;
  int next_call{0};
};

TEST_F(NavigateThroughPosesTest, SendsOrderedPosesWithDefaultActionAndRequestedTree)
{
  createServer(true);
  ToolManager manager(client);
  manager.load(config.config(Json::array({group(true)})));
  const auto started = startReady(manager);
  ASSERT_TRUE(started.at("ok").get<bool>());
  EXPECT_EQ(started.at("data").at("status"), "running");
  const auto handle = goal(0);
  EXPECT_TRUE(handle->is_active());  // 工具已返回，Action 仍在执行。
  const auto & request = *handle->get_goal();
  EXPECT_EQ(request.behavior_tree,
    "/home/yan/ros2_ws/src/scout_mini_vln/vln_behavior_tree/behavior_trees/vln_navthroughgoals.xml");
  ASSERT_EQ(request.poses.size(), 3u);
  const auto inputs = arguments().at("poses");
  for (std::size_t i = 0; i < request.poses.size(); ++i) {
    const auto & pose = request.poses[i];
    EXPECT_EQ(pose.header.frame_id, "map");
    EXPECT_EQ(pose.header.stamp, request.poses.front().header.stamp);
    EXPECT_DOUBLE_EQ(pose.pose.position.x, inputs.at(i).at("x").get<double>());
    EXPECT_DOUBLE_EQ(pose.pose.position.y, inputs.at(i).at("y").get<double>());
    EXPECT_NEAR(pose.pose.orientation.z, std::sin(inputs.at(i).at("yaw").get<double>() / 2), 1e-9);
    EXPECT_NEAR(pose.pose.orientation.w, std::cos(inputs.at(i).at("yaw").get<double>() / 2), 1e-9);
  }
  handle->succeed(std::make_shared<Action::Result>());
  expectReleased(manager, started.at("data").at("task_id"));
}

TEST_F(NavigateThroughPosesTest, ForwardsFeedbackAndConfirmsCancellationBeforeReleasing)
{
  createServer();
  auto configured = group();
  configured["settings"]["navigate_through_poses_behavior_tree"] = "/test/vln_navthroughgoals.xml";
  ToolManager manager(client);
  manager.load(config.config(Json::array({configured})));
  const auto started = startReady(manager);
  ASSERT_TRUE(started.at("ok").get<bool>());
  const auto id = started.at("data").at("task_id");
  const auto handle = goal(0);
  EXPECT_EQ(handle->get_goal()->behavior_tree, "/test/vln_navthroughgoals.xml");
  auto sample = std::make_shared<Action::Feedback>();
  sample->distance_remaining = 2.5;
  sample->navigation_time.sec = 7;
  sample->estimated_time_remaining.sec = 9;
  sample->number_of_recoveries = 3;
  sample->number_of_poses_remaining = 2;
  sample->current_pose.header.frame_id = "map";
  sample->current_pose.header.stamp.sec = 4;
  sample->current_pose.pose.position.x = 1.2;
  sample->current_pose.pose.position.y = -2.3;
  sample->current_pose.pose.orientation.z = std::sin(0.25);
  sample->current_pose.pose.orientation.w = std::cos(0.25);
  handle->publish_feedback(sample);
  ASSERT_TRUE(eventually([&] {return feedback(manager, id).at("data").contains("number_of_poses_remaining");}));
  const auto current = feedback(manager, id);
  EXPECT_DOUBLE_EQ(current.at("data").at("distance_remaining_m").get<double>(), 2.5);
  EXPECT_DOUBLE_EQ(current.at("data").at("navigation_time_sec").get<double>(), 7);
  EXPECT_DOUBLE_EQ(current.at("data").at("estimated_time_remaining_sec").get<double>(), 9);
  EXPECT_EQ(current.at("data").at("number_of_recoveries"), 3);
  EXPECT_EQ(current.at("data").at("number_of_poses_remaining"), 2);
  EXPECT_EQ(current.at("pose").at("frame_id"), "map");
  EXPECT_EQ(current.at("pose").at("time_stamp"), "1970-01-01T00:00:04.000Z");
  EXPECT_DOUBLE_EQ(current.at("pose").at("x").get<double>(), 1.2);
  EXPECT_DOUBLE_EQ(current.at("pose").at("y").get<double>(), -2.3);
  EXPECT_NEAR(current.at("pose").at("yaw").get<double>(), 0.5, 1e-9);

  reject_cancel = true;
  EXPECT_EQ(invoke(manager, "navigate_through_poses_cancel_navigation", {{"task_id", id}}).at("code"),
    "NAVIGATION_CANCEL_REJECTED");
  EXPECT_EQ(feedback(manager, id).at("data").at("status"), "running");
  reject_cancel = false;
  const auto canceled = invoke(manager, "navigate_through_poses_cancel_navigation", {{"task_id", id}});
  EXPECT_TRUE(canceled.at("ok").get<bool>());
  EXPECT_EQ(canceled.at("data").at("status"), "canceling");
  EXPECT_EQ(invoke(manager, "navigate_through_poses_cancel_navigation", {{"task_id", id}}).
    at("data").at("status"), "canceling");
  EXPECT_EQ(start(manager).at("code"), "NAVIGATION_BUSY");
  ASSERT_TRUE(eventually([&] {return handle->is_canceling();}));
  handle->canceled(std::make_shared<Action::Result>());
  expectReleased(manager, id);
  const auto next = start(manager);
  ASSERT_TRUE(next.at("ok").get<bool>());
  EXPECT_NE(next.at("data").at("task_id"), id);
  goal(1)->abort(std::make_shared<Action::Result>());
  expectReleased(manager, next.at("data").at("task_id"));
}

TEST_F(NavigateThroughPosesTest, RejectsInvalidInputsAndRecoversAfterRejectedGoals)
{
  createServer();
  ToolManager manager(client);
  manager.load(config.config(Json::array({group()})));
  auto bad_frame = arguments(); bad_frame["frame_id"] = "";
  auto empty = arguments(); empty["poses"] = Json::array();
  auto wrong_type = arguments(); wrong_type["poses"] = Json::object();
  auto missing = arguments(); missing["poses"][0].erase("yaw");
  auto nonfinite = arguments(); nonfinite["poses"][1]["x"] = std::numeric_limits<double>::infinity();
  for (const auto & invalid : {bad_frame, empty, wrong_type, missing, nonfinite}) {
    EXPECT_EQ(start(manager, invalid).at("code"), "INVALID_ARGUMENTS");
  }
  auto rejected_args = arguments(); rejected_args["poses"][0]["x"] = -1;
  const auto rejected = startReady(manager, rejected_args);
  EXPECT_EQ(rejected.at("code"), "NAVIGATION_REJECTED");
  EXPECT_EQ(rejected.at("data").at("status"), "rejected");
  EXPECT_EQ(feedback(manager, rejected.at("data").at("task_id")).at("code"), "TASK_NOT_FOUND");
  EXPECT_EQ(invoke(manager, "navigate_through_poses_cancel_navigation", {{"task_id", "missing"}}).
    at("code"), "TASK_NOT_FOUND");
  const auto next = start(manager);
  ASSERT_TRUE(next.at("ok").get<bool>());
  goal(0)->succeed(std::make_shared<Action::Result>());
  expectReleased(manager, next.at("data").at("task_id"));
}

TEST_F(NavigateThroughPosesTest, SharesBusySlotAndKeepsActionSpecificFeedbackAndCancel)
{
  createServer();
  createSingleServer();
  ToolManager manager(client);
  manager.load(config.config(Json::array({group()})));
  const auto start_single = [&] {return invoke(manager, "navigate_to_pose_start_navigation",
      {{"frame_id", "map"}, {"x", 1.0}, {"y", 2.0}, {"yaw", 0.0}});};
  Json single;
  ASSERT_TRUE(eventually([&] {single = start_single(); return single.value("ok", false);}));
  const auto single_id = single.at("data").at("task_id");
  EXPECT_EQ(start(manager).at("code"), "NAVIGATION_BUSY");
  EXPECT_EQ(feedback(manager, single_id).at("code"), "TASK_NOT_FOUND");
  EXPECT_EQ(invoke(manager, "navigate_through_poses_cancel_navigation", {{"task_id", single_id}}).
    at("code"), "TASK_NOT_FOUND");
  ASSERT_TRUE(eventually([&] {std::lock_guard<std::mutex> lock(mutex); return single_goal != nullptr;}));
  {std::lock_guard<std::mutex> lock(mutex); single_goal->succeed(std::make_shared<SingleAction::Result>());}
  Json multiple;
  ASSERT_TRUE(eventually([&] {multiple = start(manager); return multiple.value("ok", false);}));
  const auto multiple_id = multiple.at("data").at("task_id");
  EXPECT_EQ(start_single().at("code"), "NAVIGATION_BUSY");
  EXPECT_EQ(invoke(manager, "navigate_to_pose_get_feedback", {{"task_id", multiple_id}}).
    at("code"), "TASK_NOT_FOUND");
  EXPECT_EQ(invoke(manager, "navigate_to_pose_cancel_navigation", {{"task_id", multiple_id}}).
    at("code"), "TASK_NOT_FOUND");
  EXPECT_EQ(feedback(manager, multiple_id).at("data").at("status"), "running");
  goal(0)->succeed(std::make_shared<Action::Result>());
  expectReleased(manager, multiple_id);
}

TEST_F(NavigateThroughPosesTest, ReportsUnavailableServerAndCancelsActiveGoalOnShutdown)
{
  ToolManager manager(client);
  manager.load(config.config(Json::array({group()})));
  EXPECT_EQ(start(manager).at("code"), "NAVIGATION_UNAVAILABLE");
  createServer();
  const auto started = startReady(manager);
  ASSERT_TRUE(started.at("ok").get<bool>());
  const auto handle = goal(0);
  manager.shutdown();
  ASSERT_TRUE(eventually([&] {return handle->is_canceling();}));
  handle->canceled(std::make_shared<Action::Result>());
}

TEST_F(NavigateThroughPosesTest, RejectsEmptyBehaviorTreeInsteadOfUsingNav2Default)
{
  auto invalid = group();
  invalid["settings"]["navigate_through_poses_behavior_tree"] = "";
  ToolManager manager(client);
  EXPECT_THROW(manager.load(config.config(Json::array({invalid}))), std::invalid_argument);
  EXPECT_TRUE(manager.getTools().empty());
}

}  // namespace vln_agent::test
