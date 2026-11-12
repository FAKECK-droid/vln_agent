#include "test_helpers.hpp"
#include "vln_agent/utils/base64.hpp"
#include "vln_agent/srv/tool_call.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace vln_agent::test
{

/** @brief 回环专用 ROS 固件，不启动导航栈或底盘，不发布运动话题。 */
class RosToolsTest : public ::testing::Test
{
protected:
  using Action = nav2_msgs::action::NavigateToPose;
  using ServerGoal = rclcpp_action::ServerGoalHandle<Action>;

  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    client_node = std::make_shared<rclcpp::Node>("vln_agent_test_client");
    server_node = std::make_shared<rclcpp::Node>("vln_agent_test_server");
    executor.add_node(client_node);
    executor.add_node(server_node);
    spinner = std::thread([this] {executor.spin();});
    share = ament_index_cpp::get_package_share_directory("vln_agent");
    action_name = "/vln_agent_tests/navigation_" + std::to_string(getpid());
    local_costmap_name = "/vln_agent_tests/local_" + std::to_string(getpid());
    global_costmap_name = "/vln_agent_tests/global_" + std::to_string(getpid());
  }

  void TearDown() override
  {
    executor.cancel();
    spinner.join();
  }

  void createServer()
  {
    server = rclcpp_action::create_server<Action>(
      server_node, action_name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        return goal->pose.pose.position.x < 0 ?
               rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<ServerGoal>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<ServerGoal> goal) {
        std::lock_guard<std::mutex> lock(goals_mutex);
        goals.push_back(std::move(goal));
      });
    readiness_client = rclcpp_action::create_client<Action>(client_node, action_name);
    ASSERT_TRUE(readiness_client->wait_for_action_server(3s));
  }

  std::shared_ptr<ServerGoal> goal(std::size_t index)
  {
    if (!eventually([&] {
        std::lock_guard<std::mutex> lock(goals_mutex);
        return goals.size() > index;
      }))
    {
      throw std::runtime_error("假导航服务没有收到目标");
    }
    std::lock_guard<std::mutex> lock(goals_mutex);
    return goals.at(index);
  }

  Json navigationGroup()
  {
    Json functions = Json::array();
    for (const auto & suffix : {"start_navigation", "get_feedback", "cancel_navigation"}) {
      std::string class_name = std::string(suffix) == "start_navigation" ?
        "NavigateToPoseStartNavigation" : std::string(suffix) == "get_feedback" ?
        "NavigateToPoseGetFeedback" : "NavigateToPoseCancelNavigation";
      functions.push_back({
          {"name", std::string("navigate_to_pose_") + suffix},
          {"plugin", "vln_agent/" + class_name},
          {"blocking", std::string(suffix) != "get_feedback"}});
    }
    return {{"id", "navigation"},
      {"definition", share + "/plugins/tools/navigation/tool.json"},
      {"settings", {{"action_name", action_name}}},
      {"functions", functions}};
  }

  Json surroundingsGroup(std::size_t capacity = 2)
  {
    auto group = navigationGroup();
    group["settings"]["local_costmap_service"] = local_costmap_name;
    group["settings"]["global_costmap_service"] = global_costmap_name;
    group["settings"]["max_environments"] = capacity;
    group["settings"]["costmap_timeout_sec"] = 0.1;
    group["functions"].push_back({{"name", "get_surroundings"},
        {"plugin", "vln_agent/GetSurroundings"}, {"blocking", false}});
    group["functions"].push_back({{"name", "navigate_to_pixel_start_navigation"},
        {"plugin", "vln_agent/NavigateToPixelStartNavigation"}, {"blocking", true}});
    return group;
  }

  /** @brief 与普通导航共用资源，只覆盖环顾 XML 路径供假 Action 检查。 */
  Json lookaroundGroup()
  {
    auto group = surroundingsGroup();
    group["settings"]["lookaround_behavior_tree"] = "/test/behavior_trees/lookaround.xml";
    group["functions"].push_back({{"name", "start_lookaround"},
        {"plugin", "vln_agent/StartLookaround"}, {"blocking", true}});
    group["functions"].push_back({{"name", "cancel_lookaround"},
        {"plugin", "vln_agent/CancelLookaround"}, {"blocking", true}});
    return group;
  }

  void createCostmapServers()
  {
    using Service = nav2_msgs::srv::GetCostmap;
    const auto callback = [this](bool local, std::shared_ptr<Service::Response> reply) {
        ++(local ? local_reads : global_reads);
        if (empty_costmap.load()) {return;}
        auto & map = reply->map;
        map.header.frame_id = local ? "odom" : "map";
        map.metadata.resolution = 0.1;
        map.metadata.size_x = map.metadata.size_y = 200;
        map.metadata.origin.position.x = local ? -5 : 0;
        map.metadata.origin.position.y = local ? -5 : -10;
        map.metadata.origin.orientation.w = 1;
        map.data.assign(200 * 200, 0);
      };
    local_costmap = server_node->create_service<Service>(local_costmap_name,
      [callback](std::shared_ptr<Service::Request>, std::shared_ptr<Service::Response> reply) {
        callback(true, reply);
      });
    global_costmap = server_node->create_service<Service>(global_costmap_name,
      [callback](std::shared_ptr<Service::Request>, std::shared_ptr<Service::Response> reply) {
        callback(false, reply);
      });
    static_tf = std::make_shared<tf2_ros::StaticTransformBroadcaster>(server_node);
    geometry_msgs::msg::TransformStamped map_odom, odom_base;
    map_odom.header.frame_id = "map"; map_odom.child_frame_id = "odom";
    map_odom.transform.translation.x = 10; map_odom.transform.translation.y = -3;
    map_odom.transform.rotation.z = std::sin(M_PI / 4);
    map_odom.transform.rotation.w = std::cos(M_PI / 4);
    odom_base.header.frame_id = "odom"; odom_base.child_frame_id = "base_link";
    odom_base.transform.translation.x = 1; odom_base.transform.translation.y = 2;
    odom_base.transform.rotation.w = 1;
    static_tf->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{map_odom, odom_base});
  }

  Json surroundings(ToolManager & manager, const std::string & source = "local")
  {
    Json result;
    if (!eventually([&] {
        result = payload(invoke(manager, "get_surroundings", {{"costmap", source}, {"range_m", 10}}));
        return result.value("ok", false);
      }))
    {
      throw std::runtime_error("环境图未就绪: " + result.dump());
    }
    return result;
  }

  ToolCallHandle invoke(ToolManager & manager, const std::string & name, const Json & arguments)
  {
    return manager.submit(response(Json::array({
          call(name, "call_" + std::to_string(++next_call), arguments)}))).at(0);
  }

  Json start(ToolManager & manager, double x = 1.0)
  {
    return payload(invoke(manager, "navigate_to_pose_start_navigation",
        {{"frame_id", "map"}, {"x", x}, {"y", 2.0}, {"yaw", 0.5}}));
  }

  Json feedback(ToolManager & manager, const Json & id)
  {
    return payload(invoke(manager, "navigate_to_pose_get_feedback", {{"task_id", id}}));
  }

  ConfigFile config;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::Node::SharedPtr server_node;
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 2};
  std::thread spinner;
  rclcpp_action::Server<Action>::SharedPtr server;
  rclcpp_action::Client<Action>::SharedPtr readiness_client;
  std::mutex goals_mutex;
  std::vector<std::shared_ptr<ServerGoal>> goals;
  std::string share;
  std::string action_name;
  std::string local_costmap_name, global_costmap_name;
  rclcpp::Service<nav2_msgs::srv::GetCostmap>::SharedPtr local_costmap, global_costmap;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf;
  std::atomic_int local_reads{0}, global_reads{0};
  std::atomic_bool empty_costmap{false};
  int next_call{0};
};

TEST_F(RosToolsTest, LoadsInstalledDefaultConfigurationAndAllPlugins)
{
  ToolManager manager(client_node);
  manager.load(share + "/config/agent.yaml");
  const auto tools = manager.getTools();
  ASSERT_EQ(tools.size(), 9u);
  EXPECT_EQ(tools.at(0).at("name"), "navigate_to_pose_start_navigation");
  EXPECT_EQ(tools.at(3).at("name"), "get_surroundings");
  EXPECT_EQ(tools.at(4).at("name"), "navigate_to_pixel_start_navigation");
  EXPECT_EQ(tools.at(5).at("name"), "start_lookaround");
  EXPECT_EQ(tools.at(6).at("name"), "cancel_lookaround");
  EXPECT_EQ(tools.at(7).at("name"), "capture_pair");
  EXPECT_EQ(tools.at(8).at("name"), "depth_project");
  for (const auto & tool : tools) {
    EXPECT_TRUE(tool.at("async").get<bool>());
    EXPECT_TRUE(tool.at("strict").get<bool>());
    EXPECT_FALSE(tool.contains("plugin"));
    EXPECT_FALSE(tool.contains("blocking"));
  }
}

TEST_F(RosToolsTest, FailedInitializationStopsCallbackThreadsAndRollsBackPlugins)
{
  for (int attempt = 0; attempt < 3; ++attempt) {
    Json invalid_rgbd{{"id", "rgbd"},
      {"definition", share + "/plugins/tools/rgbd/tool.json"}, {"settings", Json::object()},
      {"functions", Json::array({{{"name", "capture_pair"},
          {"plugin", "vln_agent/CapturePair"}, {"blocking", false}}})}};
    ToolManager manager(client_node);
    EXPECT_THROW(manager.load(config.config(Json::array({navigationGroup(), invalid_rgbd}))),
      Json::exception);
    EXPECT_TRUE(manager.getTools().empty());
    manager.load(config.config(Json::array()));
    manager.shutdown();
  }
}

TEST_F(RosToolsTest, KeepsActiveFeedbackAndRestartsAfterCancelAndSuccessWithoutPolling)
{
  createServer();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({navigationGroup()})));
  const auto started = start(manager);
  ASSERT_TRUE(started.at("ok").get<bool>());
  EXPECT_EQ(started.at("data").at("status"), "running");
  const auto id = started.at("data").at("task_id");
  const auto first_goal = goal(0);
  EXPECT_EQ(first_goal->get_goal()->pose.header.frame_id, "map");
  EXPECT_NEAR(first_goal->get_goal()->pose.pose.orientation.z, std::sin(0.25), 1e-6);
  EXPECT_EQ(start(manager).at("code"), "NAVIGATION_BUSY");

  auto observation = std::make_shared<Action::Feedback>();
  observation->distance_remaining = 12.5;
  observation->current_pose.header.frame_id = "map";
  observation->current_pose.header.stamp.sec = 4;
  observation->current_pose.pose.position.x = 1.2;
  observation->current_pose.pose.position.y = 2.3;
  observation->current_pose.pose.orientation.z = std::sin(0.25);
  observation->current_pose.pose.orientation.w = std::cos(0.25);
  first_goal->publish_feedback(observation);
  ASSERT_TRUE(eventually([&] {return feedback(manager, id).at("data").contains("distance_remaining_m");}));
  const auto current = feedback(manager, id);
  EXPECT_NEAR(current.at("data").at("distance_remaining_m").get<double>(), 12.5, 1e-6);
  EXPECT_EQ(current.at("pose").at("frame_id"), "map");
  EXPECT_EQ(current.at("pose").at("time_stamp"), "1970-01-01T00:00:04.000Z");
  EXPECT_DOUBLE_EQ(current.at("pose").at("x").get<double>(), 1.2);
  EXPECT_DOUBLE_EQ(current.at("pose").at("y").get<double>(), 2.3);
  EXPECT_NEAR(current.at("pose").at("yaw").get<double>(), 0.5, 1e-6);

  const auto cancel = payload(invoke(manager, "navigate_to_pose_cancel_navigation", {{"task_id", id}}));
  EXPECT_TRUE(cancel.at("ok").get<bool>());
  EXPECT_EQ(cancel.at("data").at("status"), "canceling");
  EXPECT_EQ(feedback(manager, id).at("data").at("status"), "canceling");
  ASSERT_TRUE(eventually([&] {return first_goal->is_canceling();}));
  first_goal->canceled(std::make_shared<Action::Result>());
  Json next;
  ASSERT_TRUE(eventually([&] {
      next = start(manager);  // 直接启动；终态由事件处理，启动前无需读取反馈。
      return next.value("ok", false);
    }));
  EXPECT_NE(next.at("data").at("task_id"), id);
  const auto next_goal = goal(1);
  next_goal->succeed(std::make_shared<Action::Result>());
  Json after_success;
  ASSERT_TRUE(eventually([&] {
      after_success = start(manager);
      return after_success.value("ok", false);
    }));
  EXPECT_EQ(feedback(manager, id).at("code"), "TASK_NOT_FOUND");
  EXPECT_EQ(feedback(manager, next.at("data").at("task_id")).at("code"), "TASK_NOT_FOUND");
  goal(2)->abort(std::make_shared<Action::Result>());
  ASSERT_TRUE(eventually([&] {
      return feedback(manager, after_success.at("data").at("task_id")).value("code", "") == "TASK_NOT_FOUND";
    }));
}

TEST_F(RosToolsTest, RestartsAfterAbortedNavigationWithoutReadingTerminalFeedback)
{
  createServer();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({navigationGroup()})));
  const auto rejected = start(manager, -1);
  EXPECT_FALSE(rejected.at("ok").get<bool>());
  EXPECT_EQ(rejected.at("code"), "NAVIGATION_REJECTED");
  EXPECT_EQ(rejected.at("data").at("status"), "rejected");
  EXPECT_EQ(feedback(manager, "missing").at("code"), "TASK_NOT_FOUND");
  const auto accepted = start(manager);
  ASSERT_TRUE(accepted.at("ok").get<bool>());
  goal(0)->abort(std::make_shared<Action::Result>());
  Json next;
  ASSERT_TRUE(eventually([&] {
      next = start(manager);
      return next.value("ok", false);
    }));
  const auto cancel = payload(invoke(manager, "navigate_to_pose_cancel_navigation",
      {{"task_id", accepted.at("data").at("task_id")}}));
  EXPECT_EQ(cancel.at("code"), "TASK_NOT_FOUND");
  goal(1)->succeed(std::make_shared<Action::Result>());
  ASSERT_TRUE(eventually([&] {
      return feedback(manager, next.at("data").at("task_id")).value("code", "") == "TASK_NOT_FOUND";
    }));
  EXPECT_EQ(feedback(manager, accepted.at("data").at("task_id")).at("code"), "TASK_NOT_FOUND");
  EXPECT_EQ(feedback(manager, rejected.at("data").at("task_id")).at("code"), "TASK_NOT_FOUND");
}

TEST_F(RosToolsTest, UnavailableNavigationDoesNotCreateAnActiveTask)
{
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({navigationGroup()})));
  EXPECT_EQ(start(manager).at("code"), "NAVIGATION_UNAVAILABLE");
  createServer();
  ASSERT_TRUE(eventually([&] {return start(manager).value("ok", false);}));
  const auto active_goal = goal(0);
  manager.shutdown();
  ASSERT_TRUE(eventually([&] {return active_goal->is_canceling();}));
  active_goal->canceled(std::make_shared<Action::Result>());
}

TEST_F(RosToolsTest, LookaroundUsesCurrentPoseAndTreeThenReleasesSuccessAndFailureWithoutFeedback)
{
  createServer();
  createCostmapServers();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({lookaroundGroup()})));
  surroundings(manager);  // 等待专用 TF 回调准备好。

  const auto first = payload(invoke(manager, "start_lookaround", Json::object()));
  ASSERT_TRUE(first.at("ok").get<bool>());
  EXPECT_EQ(first.at("data").at("status"), "running");
  const auto first_goal = goal(0);
  const auto & request = *first_goal->get_goal();
  EXPECT_TRUE(first_goal->is_active());  // 工具已返回，行为树仍未结束。
  EXPECT_EQ(request.behavior_tree, "/test/behavior_trees/lookaround.xml");
  EXPECT_EQ(request.pose.header.frame_id, "map");
  EXPECT_NEAR(request.pose.pose.position.x, 8, 1e-6);
  EXPECT_NEAR(request.pose.pose.position.y, -2, 1e-6);
  EXPECT_NEAR(request.pose.pose.orientation.z, std::sin(M_PI / 4), 1e-6);
  EXPECT_NEAR(request.pose.pose.orientation.w, std::cos(M_PI / 4), 1e-6);
  EXPECT_EQ(start(manager).at("code"), "NAVIGATION_BUSY");
  EXPECT_EQ(payload(invoke(manager, "start_lookaround", Json::object())).at("code"),
    "NAVIGATION_BUSY");

  first_goal->succeed(std::make_shared<Action::Result>());
  Json second;
  ASSERT_TRUE(eventually([&] {
      second = payload(invoke(manager, "start_lookaround", Json::object()));
      return second.value("ok", false);
    }));
  EXPECT_NE(second.at("data").at("task_id"), first.at("data").at("task_id"));
  goal(1)->abort(std::make_shared<Action::Result>());
  Json normal;
  ASSERT_TRUE(eventually([&] {
      normal = start(manager);
      return normal.value("ok", false);
    }));
  EXPECT_EQ(goal(2)->get_goal()->behavior_tree, "");  // 不污染普通导航。
  goal(2)->succeed(std::make_shared<Action::Result>());
  ASSERT_TRUE(eventually([&] {
      return feedback(manager, normal.at("data").at("task_id")).value("code", "") == "TASK_NOT_FOUND";
    }));
}

TEST_F(RosToolsTest, CancelLookaroundUsesSharedActionAndReleasesCanceledTaskWithoutFeedback)
{
  createServer();
  createCostmapServers();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({lookaroundGroup()})));
  surroundings(manager);
  const auto first = payload(invoke(manager, "start_lookaround", Json::object()));
  ASSERT_TRUE(first.at("ok").get<bool>());
  const auto id = first.at("data").at("task_id");
  EXPECT_EQ(payload(invoke(manager, "cancel_lookaround", {{"task_id", "missing"}})).at("code"),
    "TASK_NOT_FOUND");
  const auto canceled = payload(invoke(manager, "cancel_lookaround", {{"task_id", id}}));
  ASSERT_TRUE(canceled.at("ok").get<bool>());
  EXPECT_EQ(canceled.at("data").at("task_id"), id);
  EXPECT_EQ(canceled.at("data").at("status"), "canceling");
  const auto first_goal = goal(0);
  ASSERT_TRUE(eventually([&] {return first_goal->is_canceling();}));
  first_goal->canceled(std::make_shared<Action::Result>());
  Json next;
  ASSERT_TRUE(eventually([&] {
      next = payload(invoke(manager, "start_lookaround", Json::object()));
      return next.value("ok", false);
    }));
  EXPECT_EQ(payload(invoke(manager, "cancel_lookaround", {{"task_id", id}})).at("code"),
    "TASK_NOT_FOUND");
  const auto next_goal = goal(1);
  manager.shutdown();
  ASSERT_TRUE(eventually([&] {return next_goal->is_canceling();}));
  next_goal->canceled(std::make_shared<Action::Result>());
}

TEST_F(RosToolsTest, GetsLocalAndGlobalSurroundingsWithNativePngAndSnapshotMetadata)
{
  createCostmapServers();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({surroundingsGroup()})));
  for (const auto & source : {"local", "global"}) {
    const auto result = surroundings(manager, source);
    EXPECT_EQ(result.at("images").at(0).at("metadata").at("costmap"), source);
    EXPECT_EQ(result.at("pose").at("frame_id"), "map");
    EXPECT_NEAR(result.at("pose").at("x").get<double>(), 8, 1e-6);
    EXPECT_NEAR(result.at("pose").at("y").get<double>(), -2, 1e-6);
    EXPECT_NEAR(result.at("pose").at("yaw").get<double>(), M_PI / 2, 1e-6);
    EXPECT_EQ(result.at("images").at(0).at("metadata").at("image_up"), "robot_forward");
    EXPECT_TRUE(result.at("data").contains("environment_id"));
  }
  const auto handle = invoke(manager, "get_surroundings", {{"costmap", "global"}, {"range_m", 10}});
  const auto result = payload(handle);
  const auto url = handle.result.get().at("output").at(1).at("image_url").get<std::string>();
  const std::string prefix = "data:image/png;base64,";
  ASSERT_EQ(url.substr(0, prefix.size()), prefix);
  const auto image = cv::imdecode(utils::decodeBase64(url.substr(prefix.size())), cv::IMREAD_COLOR);
  EXPECT_EQ(image.cols, result.at("images").at(0).at("metadata").at("image_width"));
  EXPECT_EQ(image.rows, result.at("images").at(0).at("metadata").at("image_height"));
  EXPECT_GT(local_reads.load(), 0); EXPECT_GT(global_reads.load(), 0);
}

TEST_F(RosToolsTest, PixelNavigationUsesSnapshotPositionAndImageRelativeYaw)
{
  createServer();
  createCostmapServers();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({surroundingsGroup()})));
  surroundings(manager);  // 等待 TF 准备完毕；正式快照由机器人经服务获取。
  const auto robot = client_node->create_client<srv::ToolCall>("/vln/tool_call");
  ASSERT_TRUE(robot->wait_for_service(3s));
  auto request = std::make_shared<srv::ToolCall::Request>();
  request->request_json = response(Json::array({
      call("get_surroundings", "robot_environment", {{"costmap", "local"}, {"range_m", 10}})})).dump();
  auto future = robot->async_send_request(request);
  ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
  const auto reply = future.get();
  ASSERT_TRUE(reply->error.empty());
  const auto outputs = Json::parse(reply->outputs_json);
  const auto environment = Json::parse(outputs.at(0).at("output").at(0).at("text").get<std::string>());
  const auto & rect = environment.at("images").at(0).at("metadata").at("map_rect");
  const double x = rect.at("x").get<double>() + (rect.at("width").get<double>() - 1) / 2;
  const double y = rect.at("y").get<double>() + (rect.at("height").get<double>() - 1) / 2 -
    1 / environment.at("images").at(0).at("metadata").at("meters_per_pixel").get<double>();
  const Json arguments{{"environment_id", environment.at("data").at("environment_id")},
    {"pixel_x", x}, {"pixel_y", y}, {"yaw", M_PI / 4}};
  const auto result = payload(invoke(manager, "navigate_to_pixel_start_navigation", arguments));
  ASSERT_TRUE(result.at("ok").get<bool>());
  EXPECT_EQ(result.at("data").at("environment_id"), environment.at("data").at("environment_id"));
  EXPECT_TRUE(result.at("data").contains("task_id"));
  const auto accepted = goal(0);
  const auto & pose = accepted->get_goal()->pose;
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_NEAR(pose.pose.position.x, 8, 1e-6);
  EXPECT_NEAR(pose.pose.position.y, -1, 1e-6);
  EXPECT_NEAR(pose.pose.orientation.z, std::sin(3 * M_PI / 8), 1e-6);
  EXPECT_NEAR(pose.pose.orientation.w, std::cos(3 * M_PI / 8), 1e-6);
  EXPECT_EQ(payload(invoke(manager, "navigate_to_pixel_start_navigation", arguments)).
    at("code"), "ENVIRONMENT_NOT_FOUND");
  accepted->succeed(std::make_shared<Action::Result>());
  ASSERT_TRUE(eventually([&] {
      return feedback(manager, result.at("data").at("task_id")).value("code", "") == "TASK_NOT_FOUND";
    }));
}

TEST_F(RosToolsTest, EnvironmentCacheIsBoundedAndRejectsPixelsOutsideMap)
{
  createCostmapServers();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({surroundingsGroup(2)})));
  const auto first = surroundings(manager);
  surroundings(manager);
  const auto latest = surroundings(manager);
  const auto expired = payload(invoke(manager, "navigate_to_pixel_start_navigation",
      {{"environment_id", first.at("data").at("environment_id")}, {"pixel_x", 300}, {"pixel_y", 300}, {"yaw", 0}}));
  EXPECT_EQ(expired.at("code"), "ENVIRONMENT_NOT_FOUND");
  const auto outside = payload(invoke(manager, "navigate_to_pixel_start_navigation",
      {{"environment_id", latest.at("data").at("environment_id")}, {"pixel_x", 0}, {"pixel_y", 0}, {"yaw", 0}}));
  EXPECT_FALSE(outside.at("ok").get<bool>());
}

TEST_F(RosToolsTest, ReportsUnavailableAndEmptyCostmaps)
{
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({surroundingsGroup()})));
  EXPECT_EQ(payload(invoke(manager, "get_surroundings", {{"costmap", "local"}, {"range_m", 10}})).
    at("code"), "COSTMAP_UNAVAILABLE");
  empty_costmap = true;
  createCostmapServers();
  EXPECT_EQ(payload(invoke(manager, "get_surroundings", {{"costmap", "local"}, {"range_m", 10}})).
    at("code"), "EMPTY_COSTMAP");
}

}  // namespace vln_agent::test
