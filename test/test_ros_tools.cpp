#include "test_helpers.hpp"
#include "vln_agent/utils/base64.hpp"

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
#include <rgbd_image_queue/srv/get_image.hpp>

namespace vln_agent::test
{

/** @brief 回环专用 ROS 固件，不启动导航栈或底盘，不发布运动话题。 */
class RosToolsTest : public ::testing::Test
{
protected:
  using Action = nav2_msgs::action::NavigateToPose;
  using ServerGoal = rclcpp_action::ServerGoalHandle<Action>;
  using GetImage = rgbd_image_queue::srv::GetImage;

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
    image_service_name = "/vln_agent_tests/image_" + std::to_string(getpid());
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

  void createImageServer()
  {
    image_server = server_node->create_service<GetImage>(image_service_name,
      [this](const rclcpp::Service<GetImage>::SharedPtr service,
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<GetImage::Request> request)
      {
        {
          std::lock_guard<std::mutex> lock(image_requests_mutex);
          image_requests.emplace_back(request->operation, request->detail);
        }
        if (withhold_image.load()) {return;}
        GetImage::Response reply;
        reply.success = !empty_queue.load();
        reply.message = reply.success ? "ok" : "selected cache is empty";
        reply.has_color = reply.success;
        reply.color.header.frame_id = "test_camera";
        reply.color.header.stamp.sec = 4;
        reply.color.header.stamp.nanosec = 5;
        reply.color.format = request->operation == GetImage::Request::QUEUE_COLOR_AT_AGE ?
          "rgb8; png compressed bgr8" : "rgb8; jpeg compressed bgr8";
        // 不透明载荷用于确认消费者没有尝试解码或重新编码。
        reply.color.data = {0xff, 0xd8, 1, 2, 3, 0xff, 0xd9};
        reply.pair_id = 100 + request->detail;
        reply.queue_index = request->operation == GetImage::Request::LATEST_COLOR ? -1 :
          (request->operation == GetImage::Request::QUEUE_COLOR ?
          static_cast<std::int64_t>(request->detail) : 1);
        reply.queue_size = 3;
        reply.image_age_sec = 1.625;
        reply.age_error_sec = request->operation == GetImage::Request::QUEUE_COLOR_AT_AGE ?
          0.125 : 0.0;
        reply.exceeded_max_age = true;
        service->send_response(*header, reply);
      });
    const auto readiness = client_node->create_client<GetImage>(image_service_name);
    ASSERT_TRUE(readiness->wait_for_service(3s));
  }

  Json cameraGroup(double timeout_sec = 1.0)
  {
    return {{"id", "camera"}, {"definition", share + "/plugins/tools/capture_image/tool.json"},
      {"settings", {{"service_name", image_service_name}}},
      {"functions", Json::array({{{"name", "capture_image"},
            {"plugin", "vln_agent/CaptureImage"}, {"blocking", false},
            {"settings", {{"timeout_sec", timeout_sec}}}}})}};
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
  std::string image_service_name;
  rclcpp::Service<GetImage>::SharedPtr image_server;
  std::mutex image_requests_mutex;
  std::vector<std::pair<std::uint8_t, std::uint32_t>> image_requests;
  std::atomic_bool empty_queue{false}, withhold_image{false};
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
  ASSERT_EQ(tools.size(), 6u);
  EXPECT_EQ(tools.at(0).at("name"), "navigate_to_pose_start_navigation");
  EXPECT_EQ(tools.at(3).at("name"), "get_surroundings");
  EXPECT_EQ(tools.at(4).at("name"), "navigate_to_pixel_start_navigation");
  EXPECT_EQ(tools.at(5).at("name"), "capture_image");
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

TEST_F(RosToolsTest, SharesCurrentNavigationAndConsumesTerminalStateOnce)
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
  EXPECT_EQ(feedback(manager, id).at("code"), "TASK_NOT_FOUND");
  EXPECT_EQ(feedback(manager, next.at("task_id")).at("code"), "TASK_NOT_FOUND");
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
  // 未交付的终态仍有用途：先读取它，再允许启动下一个导航。
  ASSERT_TRUE(eventually([&] {
      return start(manager).value("code", std::string()) == "NAVIGATION_RESULT_PENDING";
    }));
  const auto cancel = payload(invoke(manager, "navigate_to_pose_cancel_navigation",
      {{"task_id", accepted.at("task_id")}}));
  EXPECT_TRUE(cancel.at("ok").get<bool>());
  EXPECT_EQ(cancel.at("status"), "failed");
  EXPECT_EQ(feedback(manager, accepted.at("task_id")).at("code"), "TASK_NOT_FOUND");
  EXPECT_EQ(feedback(manager, rejected.at("task_id")).at("code"), "TASK_NOT_FOUND");
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

TEST_F(RosToolsTest, ReportsUnavailableImageService)
{
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({cameraGroup(0.03)})));
  const auto handle = invoke(manager, "capture_image", {{"mode", "latest"}, {"detail", 0}});
  EXPECT_EQ(payload(handle).at("code"), "IMAGE_SERVICE_UNAVAILABLE");
  EXPECT_EQ(handle.result.get().at("output").size(), 1u);
}

TEST_F(RosToolsTest, ForwardsLatestIndexAndAgeRequestsWithoutTranscoding)
{
  createImageServer();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({cameraGroup()})));
  const std::vector<Json> arguments{
    {{"mode", "latest"}, {"detail", 0}},
    {{"mode", "index"}, {"detail", 2}},
    {{"mode", "age"}, {"detail", 1500}}};
  std::vector<ToolCallHandle> handles;
  for (const auto & argument : arguments) {
    handles.push_back(invoke(manager, "capture_image", argument));
  }
  for (std::size_t i = 0; i < handles.size(); ++i) {
    const auto result = payload(handles[i]);
    ASSERT_TRUE(result.at("ok").get<bool>());
    EXPECT_EQ(result.at("frame_id"), "test_camera");
    EXPECT_EQ(result.at("image_time_stamp"), "1970-01-01T00:00:04.000Z");
    EXPECT_EQ(result.at("pair_id"), 100 + arguments[i].at("detail").get<int>());
    EXPECT_EQ(result.at("queue_index"), i == 0 ? -1 : (i == 1 ? 2 : 1));
    EXPECT_EQ(result.at("queue_size"), 3);
    EXPECT_DOUBLE_EQ(result.at("image_age_sec").get<double>(), 1.625);
    EXPECT_DOUBLE_EQ(result.at("age_error_sec").get<double>(), i == 2 ? 0.125 : 0.0);
    EXPECT_TRUE(result.at("exceeded_max_age").get<bool>());
    EXPECT_TRUE(result.contains("time_stamp"));
    EXPECT_FALSE(result.contains("width"));

    const auto contents = handles[i].result.get().at("output");
    ASSERT_EQ(contents.size(), 2u);
    EXPECT_EQ(contents.at(1).at("type"), "input_image");
    const std::string prefix = i == 2 ? "data:image/png;base64," : "data:image/jpeg;base64,";
    const std::vector<unsigned char> original{0xff, 0xd8, 1, 2, 3, 0xff, 0xd9};
    EXPECT_EQ(contents.at(1).at("image_url"), prefix + utils::encodeBase64(original));
  }
  std::lock_guard<std::mutex> lock(image_requests_mutex);
  ASSERT_EQ(image_requests.size(), 3u);
  for (const auto & expected : std::vector<std::pair<std::uint8_t, std::uint32_t>>{
      {GetImage::Request::LATEST_COLOR, 0}, {GetImage::Request::QUEUE_COLOR, 2},
      {GetImage::Request::QUEUE_COLOR_AT_AGE, 1500}})
  {
    EXPECT_NE(std::find(image_requests.begin(), image_requests.end(), expected), image_requests.end());
  }
}

TEST_F(RosToolsTest, ReportsImageServiceFailureWithoutFallback)
{
  empty_queue = true;
  createImageServer();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({cameraGroup()})));
  const auto handle = invoke(manager, "capture_image", {{"mode", "index"}, {"detail", 2}});
  const auto result = payload(handle);
  EXPECT_FALSE(result.at("ok").get<bool>());
  EXPECT_EQ(result.at("code"), "IMAGE_UNAVAILABLE");
  EXPECT_EQ(result.at("message"), "selected cache is empty");
  EXPECT_EQ(handle.result.get().at("output").size(), 1u);
  std::lock_guard<std::mutex> lock(image_requests_mutex);
  EXPECT_EQ(image_requests.size(), 1u);
}

TEST_F(RosToolsTest, TimesOutUnansweredImageRequestsAndCanReadAgain)
{
  withhold_image = true;
  createImageServer();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({cameraGroup(0.03)})));
  for (int i = 0; i < 2; ++i) {
    const auto result = payload(invoke(manager, "capture_image",
        {{"mode", "latest"}, {"detail", 0}}));
    EXPECT_EQ(result.at("code"), "IMAGE_SERVICE_TIMEOUT");
  }
  withhold_image = false;
  const auto recovered = payload(invoke(manager, "capture_image",
      {{"mode", "index"}, {"detail", 1}}));
  EXPECT_TRUE(recovered.at("ok").get<bool>());
}

TEST_F(RosToolsTest, ShutdownReleasesPendingImageServiceRequests)
{
  withhold_image = true;
  createImageServer();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({cameraGroup(10.0)})));
  const auto handle = invoke(manager, "capture_image", {{"mode", "latest"}, {"detail", 0}});
  ASSERT_TRUE(eventually([&] {
      std::lock_guard<std::mutex> lock(image_requests_mutex);
      return !image_requests.empty();
    }));
  const auto started = std::chrono::steady_clock::now();
  manager.shutdown();
  EXPECT_LT(std::chrono::steady_clock::now() - started, 1s);
  EXPECT_EQ(payload(handle).at("code"), "TOOL_MANAGER_CLOSED");
  EXPECT_TRUE(manager.getTools().empty());
}

TEST_F(RosToolsTest, GetsLocalAndGlobalSurroundingsWithNativePngAndSnapshotMetadata)
{
  createCostmapServers();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({surroundingsGroup()})));
  for (const auto & source : {"local", "global"}) {
    const auto result = surroundings(manager, source);
    EXPECT_EQ(result.at("costmap"), source);
    EXPECT_EQ(result.at("frame_id"), "map");
    EXPECT_NEAR(result.at("robot_pose").at("x").get<double>(), 8, 1e-6);
    EXPECT_NEAR(result.at("robot_pose").at("y").get<double>(), -2, 1e-6);
    EXPECT_NEAR(result.at("robot_pose").at("yaw").get<double>(), M_PI / 2, 1e-6);
    EXPECT_EQ(result.at("image_up"), "robot_forward");
    EXPECT_TRUE(result.contains("environment_id"));
  }
  const auto handle = invoke(manager, "get_surroundings", {{"costmap", "global"}, {"range_m", 10}});
  const auto result = payload(handle);
  const auto url = handle.result.get().at("output").at(1).at("image_url").get<std::string>();
  const std::string prefix = "data:image/png;base64,";
  ASSERT_EQ(url.substr(0, prefix.size()), prefix);
  const auto image = cv::imdecode(utils::decodeBase64(url.substr(prefix.size())), cv::IMREAD_COLOR);
  EXPECT_EQ(image.cols, result.at("image_width"));
  EXPECT_EQ(image.rows, result.at("image_height"));
  EXPECT_GT(local_reads.load(), 0); EXPECT_GT(global_reads.load(), 0);
}

TEST_F(RosToolsTest, PixelNavigationUsesSnapshotPositionAndImageRelativeYaw)
{
  createServer();
  createCostmapServers();
  ToolManager manager(client_node);
  manager.load(config.config(Json::array({surroundingsGroup()})));
  const auto environment = surroundings(manager);
  const auto & rect = environment.at("map_rect");
  const double x = rect.at("x").get<double>() + (rect.at("width").get<double>() - 1) / 2;
  const double y = rect.at("y").get<double>() + (rect.at("height").get<double>() - 1) / 2 -
    1 / environment.at("meters_per_pixel").get<double>();
  const Json arguments{{"environment_id", environment.at("environment_id")},
    {"pixel_x", x}, {"pixel_y", y}, {"yaw", M_PI / 4}};
  const auto result = payload(invoke(manager, "navigate_to_pixel_start_navigation", arguments));
  ASSERT_TRUE(result.at("ok").get<bool>());
  EXPECT_EQ(result.at("environment_id"), environment.at("environment_id"));
  EXPECT_TRUE(result.contains("task_id"));
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
      return feedback(manager, result.at("task_id")).value("status", "") == "succeeded";
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
      {{"environment_id", first.at("environment_id")}, {"pixel_x", 300}, {"pixel_y", 300}, {"yaw", 0}}));
  EXPECT_EQ(expired.at("code"), "ENVIRONMENT_NOT_FOUND");
  const auto outside = payload(invoke(manager, "navigate_to_pixel_start_navigation",
      {{"environment_id", latest.at("environment_id")}, {"pixel_x", 0}, {"pixel_y", 0}, {"yaw", 0}}));
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
