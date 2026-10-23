#include "test_helpers.hpp"
#include "vln_agent/utils/base64.hpp"

#include <cmath>
#include <condition_variable>
#include <mutex>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>

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
    camera_topic = "/vln_agent_tests/image_" + std::to_string(getpid());
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
      {"definition", share + "/plugins/tools/navigate_to_pose/tool.json"},
      {"settings", {{"action_name", action_name}}},
      {"functions", functions}};
  }

  Json cameraGroup(double timeout_sec = 0.1)
  {
    return {{"id", "camera"}, {"definition", share + "/plugins/tools/capture_image/tool.json"},
      {"settings", {{"camera_topic", camera_topic}}},
      {"functions", Json::array({{{"name", "capture_image"},
            {"plugin", "vln_agent/CaptureImage"}, {"blocking", false},
            {"settings", {{"timeout_sec", timeout_sec}}}}})}};
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
  std::string camera_topic;
  int next_call{0};
};

TEST_F(RosToolsTest, LoadsInstalledDefaultConfigurationAndAllPlugins)
{
  ToolManager manager(client_node);
  manager.load(share + "/config/agent.yaml");
  const auto tools = manager.getTools();
  ASSERT_EQ(tools.size(), 4u);
  EXPECT_EQ(tools.at(0).at("name"), "navigate_to_pose_start_navigation");
  EXPECT_EQ(tools.at(3).at("name"), "capture_image");
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
    auto invalid_camera = cameraGroup();
    invalid_camera["settings"] = Json::object();
    ToolManager manager(client_node);
    EXPECT_THROW(manager.load(config.config(Json::array({navigationGroup(), invalid_camera}))),
      Json::exception);
    EXPECT_TRUE(manager.getTools().empty());
    manager.load(config.config(Json::array()));
    manager.shutdown();
  }
}

TEST_F(RosToolsTest, SharesNavigationStateAndKeepsCanceledHistorySeparateFromNewGoal)
{
  createServer();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({navigationGroup()})));
  const auto started = start(manager);
  ASSERT_TRUE(started.at("ok").get<bool>());
  EXPECT_EQ(started.at("status"), "running");
  const auto id = started.at("task_id");
  const auto first_goal = goal(0);
  EXPECT_EQ(first_goal->get_goal()->pose.header.frame_id, "map");
  EXPECT_NEAR(first_goal->get_goal()->pose.pose.orientation.z, std::sin(0.25), 1e-6);
  EXPECT_EQ(start(manager).at("code"), "NAVIGATION_BUSY");

  auto observation = std::make_shared<Action::Feedback>();
  observation->distance_remaining = 12.5;
  first_goal->publish_feedback(observation);
  ASSERT_TRUE(eventually([&] {return feedback(manager, id).contains("distance_remaining_m");}));
  EXPECT_NEAR(feedback(manager, id).at("distance_remaining_m").get<double>(), 12.5, 1e-6);

  const auto cancel = payload(invoke(manager, "navigate_to_pose_cancel_navigation", {{"task_id", id}}));
  EXPECT_TRUE(cancel.at("ok").get<bool>());
  EXPECT_EQ(cancel.at("status"), "canceling");
  EXPECT_EQ(feedback(manager, id).at("status"), "canceling");
  ASSERT_TRUE(eventually([&] {return first_goal->is_canceling();}));
  first_goal->canceled(std::make_shared<Action::Result>());
  ASSERT_TRUE(eventually([&] {return feedback(manager, id).at("status") == "canceled";}));

  const auto next = start(manager);
  ASSERT_TRUE(next.at("ok").get<bool>());
  EXPECT_NE(next.at("task_id"), id);
  const auto next_goal = goal(1);
  next_goal->succeed(std::make_shared<Action::Result>());
  ASSERT_TRUE(eventually([&] {
      return feedback(manager, next.at("task_id")).at("status") == "succeeded";
    }));
  EXPECT_EQ(feedback(manager, id).at("status"), "canceled");
}

TEST_F(RosToolsTest, ReportsRejectionUnknownTaskAndFailedPhysicalResult)
{
  createServer();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({navigationGroup()})));
  const auto rejected = start(manager, -1);
  EXPECT_FALSE(rejected.at("ok").get<bool>());
  EXPECT_EQ(rejected.at("code"), "NAVIGATION_REJECTED");
  EXPECT_EQ(rejected.at("status"), "rejected");
  EXPECT_EQ(feedback(manager, "missing").at("code"), "TASK_NOT_FOUND");
  const auto accepted = start(manager);
  ASSERT_TRUE(accepted.at("ok").get<bool>());
  goal(0)->abort(std::make_shared<Action::Result>());
  ASSERT_TRUE(eventually([&] {
      return feedback(manager, accepted.at("task_id")).at("status") == "failed";
    }));
  EXPECT_TRUE(feedback(manager, accepted.at("task_id")).at("ok").get<bool>());
  const auto cancel = payload(invoke(manager, "navigate_to_pose_cancel_navigation",
      {{"task_id", accepted.at("task_id")}}));
  EXPECT_TRUE(cancel.at("ok").get<bool>());
  EXPECT_EQ(cancel.at("status"), "failed");
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

TEST_F(RosToolsTest, CapturesSyntheticImageAsNativeJpegAndTimesOutWithoutFirstFrame)
{
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({cameraGroup()})));
  EXPECT_EQ(payload(invoke(manager, "capture_image", Json::object())).at("code"), "IMAGE_TIMEOUT");

  const auto publisher = server_node->create_publisher<sensor_msgs::msg::Image>(
    camera_topic, rclcpp::SensorDataQoS());
  ASSERT_TRUE(eventually([&] {return publisher->get_subscription_count() > 0;}));
  sensor_msgs::msg::Image image;
  image.header.frame_id = "test_camera";
  image.header.stamp.sec = 4;
  image.header.stamp.nanosec = 5;
  image.width = 2;
  image.height = 2;
  image.encoding = "bgr8";
  image.step = 6;
  image.data = {0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255};
  publisher->publish(image);
  const auto handle = invoke(manager, "capture_image", Json::object());
  const auto result = payload(handle);
  ASSERT_TRUE(result.at("ok").get<bool>());
  EXPECT_EQ(result.at("frame_id"), "test_camera");
  EXPECT_EQ(result.at("image_time_stamp"), "1970-01-01T00:00:04.000Z");
  EXPECT_TRUE(result.contains("time_stamp"));
  const auto contents = handle.result.get().at("output");
  ASSERT_EQ(contents.size(), 2u);
  EXPECT_EQ(contents.at(1).at("type"), "input_image");
  const auto url = contents.at(1).at("image_url").get<std::string>();
  const std::string prefix = "data:image/jpeg;base64,";
  ASSERT_EQ(url.substr(0, prefix.size()), prefix);
  const auto decoded = cv::imdecode(utils::decodeBase64(url.substr(prefix.size())), cv::IMREAD_COLOR);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.rows, 2);
  EXPECT_EQ(decoded.cols, 2);
  EXPECT_GT(decoded.at<cv::Vec3b>(0, 0)[2], 200);
}

TEST_F(RosToolsTest, ShutdownWakesImageWaitAndReleasesCallbacksBeforeExecutorStops)
{
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({cameraGroup(10.0)})));
  const auto handle = invoke(manager, "capture_image", Json::object());
  EXPECT_EQ(handle.result.wait_for(20ms), std::future_status::timeout);
  manager.shutdown();
  EXPECT_EQ(payload(handle).at("code"), "TOOL_MANAGER_CLOSED");
  EXPECT_TRUE(manager.getTools().empty());
}

}  // namespace vln_agent::test

