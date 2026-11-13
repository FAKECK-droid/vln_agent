#include "test_helpers.hpp"
#include "vln_agent/plugins/tools/rgbd/utils.hpp"
#include "vln_agent/utils/base64.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <cstdlib>
#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rgbd_image_queue/srv/get_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace vln_agent::test
{
namespace
{
using GetImage = rgbd_image_queue::srv::GetImage;

/** @brief 构造真实 ROS compressedDepth PNG 包，包含 12 字节 ConfigHeader。 */
sensor_msgs::msg::CompressedImage compressedDepth(bool inverse = false, int width = 8, int height = 6)
{
  cv::Mat pixels(height, width, CV_16UC1, cv::Scalar(inverse ? 510 : 2000));
  pixels.at<std::uint16_t>(0, 0) = 0;
  std::vector<unsigned char> png;
  cv::imencode(".png", pixels, png);
  sensor_msgs::msg::CompressedImage image;
  image.header.frame_id = "rgbd_test_optical";
  image.header.stamp.sec = 4;
  image.format = inverse ? "32FC1; compressedDepth png" : "16UC1; compressedDepth png";
  image.data.resize(12 + png.size());
  const std::array<float, 2> parameters{1000, 10};
  std::memcpy(image.data.data() + 4, parameters.data(), sizeof(parameters));
  std::copy(png.begin(), png.end(), image.data.begin() + 12);
  return image;
}
}  // namespace

TEST(RgbdMathTest, DecodesMillimetersAndInverseDepthWithInvalidAndOutsidePixels)
{
  for (const bool inverse : {false, true}) {
    const auto depth = rgbd::decodeDepth(compressedDepth(inverse));
    EXPECT_DOUBLE_EQ(rgbd::depthAt(depth, 5, 4), 2.0);
    EXPECT_THROW(rgbd::depthAt(depth, 0, 0), std::invalid_argument);
    EXPECT_THROW(rgbd::depthAt(depth, 8, 1), std::out_of_range);
    EXPECT_THROW(rgbd::depthAt(depth, -1, 1), std::out_of_range);
  }
  auto bad = compressedDepth();
  bad.format = "16UC1; compressedDepth rvl";
  EXPECT_THROW(rgbd::decodeDepth(bad), std::invalid_argument);
  bad.format = "16UC1; compressedDepth png";
  bad.data.resize(12);
  EXPECT_THROW(rgbd::decodeDepth(bad), std::invalid_argument);
}

TEST(RgbdMathTest, MapsReturnedPixelCentersUsingActualRoundedDimensions)
{
  EXPECT_EQ(rgbd::toDepthPixel(2, 1, {4, 3}, {8, 6}), cv::Point(5, 3));
  EXPECT_EQ(rgbd::toDepthPixel(0, 0, {4, 3}, {8, 6}), cv::Point(1, 1));
  EXPECT_EQ(rgbd::toDepthPixel(3, 2, {4, 3}, {8, 6}), cv::Point(7, 5));
  // 0.4 * 8/6 的实际返回尺寸为 3x2，查询不能直接除以配置中的 0.4。
  EXPECT_EQ(rgbd::toDepthPixel(1, 1, {3, 2}, {8, 6}), cv::Point(4, 4));
  EXPECT_EQ(rgbd::toDepthPixel(11, 8, {12, 9}, {8, 6}), cv::Point(7, 5));
  EXPECT_EQ(rgbd::toDepthPixel(5, 4, {8, 6}, {8, 6}), cv::Point(5, 4));
  EXPECT_EQ(rgbd::toDepthPixel(0, 0, {1, 1}, {8, 6}), cv::Point(4, 3));
  EXPECT_THROW(rgbd::toDepthPixel(4, 1, {4, 3}, {8, 6}), std::out_of_range);
  EXPECT_THROW(rgbd::toDepthPixel(-1, 1, {4, 3}, {8, 6}), std::out_of_range);
}

TEST(RgbdMathTest, RemovesRawImageDistortionBeforeBackProjection)
{
  sensor_msgs::msg::CameraInfo info;
  info.k = {100, 0, 0, 0, 100, 0, 0, 0, 1};
  info.distortion_model = "plumb_bob";
  info.d = {0.08, 0, 0, 0, 0};
  // 归一化 x=0.5 经径向畸变后为 0.51，对应原始像素 u=51。
  const auto pinhole = rgbd::backProject(info, 51, 0, 2);
  EXPECT_NEAR(pinhole.x, 1.0, 1e-7);
  EXPECT_NEAR(pinhole.y, 0, 1e-9);
  EXPECT_DOUBLE_EQ(pinhole.z, 2);
  info.distortion_model = "equidistant";
  info.d = {0, 0, 0, 0};
  const auto fisheye = rgbd::backProject(info, 50, 0, 2);
  EXPECT_NEAR(fisheye.x, 2 * std::tan(0.5), 1e-9);
}

/** @brief 仅运行合成 GetImage、CameraInfo 和静态 TF，不连接相机或机器人。 */
class RgbdToolsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    host = std::make_shared<rclcpp::Node>("rgbd_tools_test");
    source = std::make_shared<rclcpp::Node>("rgbd_test_source");
    executor.add_node(source);
    spinner = std::thread([this] {executor.spin();});
    // 立即跳过的用例也须先启动 spin，避免 cancel 早于 spin 被覆盖而卡在 join。
    ASSERT_TRUE(eventually([this] {return executor.is_spinning();}));
    service_name = "/rgbd_test/get_image_" + std::to_string(getpid());
    info_name = "/rgbd_test/camera_info_" + std::to_string(getpid());
    share = ament_index_cpp::get_package_share_directory("vln_agent");
  }
  void TearDown() override
  {
    executor.cancel();
    spinner.join();
  }
  Json group(std::size_t capacity = 8, double timeout = 0.3, double scale = 1.0)
  {
    return {{"id", "rgbd"}, {"definition", share + "/plugins/tools/rgbd/tool.json"},
      {"settings", {{"service_name", service_name}, {"camera_info_topic", info_name},
          {"fixed_frame", "rgbd_test_map"}, {"max_pairs", capacity}, {"timeout_sec", timeout},
          {"segmentation", {{"engine_directory", std::getenv("VLN_SAM_ENGINE_DIR") ?
              std::getenv("VLN_SAM_ENGINE_DIR") : ""}}}}},
      {"functions", Json::array({
          {{"name", "capture_pair"}, {"plugin", "vln_agent/CapturePair"}, {"blocking", false},
            {"settings", {{"image_scale", scale}}}},
          {{"name", "depth_project"}, {"plugin", "vln_agent/DepthProject"}, {"blocking", false}},
          {{"name", "depth_project_surround"}, {"plugin", "vln_agent/DepthProjectSurround"}, {"blocking", false}}})}};
  }

  void createSource(bool with_calibration = true)
  {
    depth = compressedDepth(false, width, height);
    cv::imencode(".jpg", cv::Mat(height, width, CV_8UC3, cv::Scalar(20, 80, 180)), color);
    server = source->create_service<GetImage>(service_name,
      [this](GetImage::Request::SharedPtr request, GetImage::Response::SharedPtr reply) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          requests.emplace_back(request->operation, request->detail);
        }
        if (request->operation == GetImage::Request::STATUS) {
          std::this_thread::sleep_for(std::chrono::milliseconds(status_delay_ms.load()));
          reply->success = !status_failure;
          reply->message = "test status failure";
          reply->latest_available = !missing_pair;
          reply->latest_age_sec = 0.25;
          return;
        }
        reply->success = !missing_pair;
        reply->message = missing_pair ? "no paired image" : "ok";
        reply->has_color = reply->has_depth = !missing_pair;
        reply->pair_id = 200 + request->detail;
        reply->color.header = depth.header;
        reply->color.format = color_format;
        reply->color.data = color;
        reply->depth = depth;
      });
    broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(source);
    sendTransforms(10, 20);
    if (with_calibration) {
      info_publisher = source->create_publisher<sensor_msgs::msg::CameraInfo>(info_name, 1);
      info_timer = source->create_wall_timer(10ms, [this] {
        sensor_msgs::msg::CameraInfo info;
        info.header.frame_id = depth.header.frame_id;
        info.width = width; info.height = height;
        info.k = {4, 0, 3, 0, 5, 2, 0, 0, 1};
        info.distortion_model = "plumb_bob";
        info.d = {0, 0, 0, 0, 0};
        info_publisher->publish(info);
      });
    }
  }
  void sendTransforms(double camera_x, double camera_y)
  {
    geometry_msgs::msg::TransformStamped camera, base;
    camera.header.frame_id = base.header.frame_id = "rgbd_test_map";
    camera.child_frame_id = "rgbd_test_optical";
    camera.transform.translation.x = camera_x;
    camera.transform.translation.y = camera_y;
    camera.transform.translation.z = 1;
    camera.transform.rotation.z = camera.transform.rotation.w = std::sqrt(0.5);
    base.child_frame_id = "rgbd_test_base";
    base.transform.translation.x = 6; base.transform.translation.y = 7;
    base.transform.rotation.w = 1;
    broadcaster->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{camera, base});
  }
  ToolCallHandle invoke(ToolManager & manager, const std::string & name, const Json & args)
  {
    return manager.submit(response(Json::array({
          call(name, "rgbd_" + std::to_string(++sequence), args)}))).at(0);
  }
  ToolCallHandle capture(ToolManager & manager, const std::string & mode = "latest", int detail = 0)
  {
    ToolCallHandle handle;
    Json result;
    if (!eventually([&] {
        handle = invoke(manager, "capture_pair", {{"mode", mode}, {"detail", detail}});
        result = payload(handle);
        return result.value("ok", false);
      }))
    {
      throw std::runtime_error("图像对未就绪: " + result.dump());
    }
    return handle;
  }
  Json instance(int x, int y, int label = 1)
  {
    return {{"box", nullptr}, {"points", Json::array({{{"x", x}, {"y", y}, {"label", label}}})}};
  }
  Json project(ToolManager & manager, const Json & id, const Json & instances,
    const std::string & frame = "rgbd_test_map", bool segmentation = false)
  {
    return payload(invoke(manager, "depth_project", {{"pair_id", id}, {"target_frame", frame},
        {"use_segmentation", segmentation}, {"instances", instances}}));
  }
  Json query(ToolManager & manager, const Json & id, int x = 5, int y = 4,
    const std::string & frame = "rgbd_test_map")
  {
    return project(manager, id, Json::array({instance(x, y)}), frame);
  }

  ConfigFile config;
  rclcpp::Node::SharedPtr host, source;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spinner;
  std::string service_name, info_name, share;
  rclcpp::Service<GetImage>::SharedPtr server;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher;
  rclcpp::TimerBase::SharedPtr info_timer;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
  sensor_msgs::msg::CompressedImage depth;
  std::vector<unsigned char> color;
  std::string color_format{"jpeg"};
  std::mutex mutex;
  std::vector<std::pair<std::uint8_t, std::uint32_t>> requests;
  std::atomic_bool missing_pair{false}, status_failure{false};
  std::atomic_int status_delay_ms{0};
  int sequence{0};
  int width{8}, height{6};
};


/** @brief 相机时效只读 STATUS；空帧、错误、超时均不丢失导航/TF 状态。 */
TEST_F(RgbdToolsTest, RobotStateReadsOnlyCameraStatusAndKeepsPartialResults)
{
  const Json navigation{{"id", "navigation"},
    {"definition", share + "/plugins/tools/navigation/tool.json"},
    {"settings", {{"map_frame", "rgbd_test_map"}, {"base_frame", "rgbd_test_base"}}},
    {"functions", Json::array({{{"name", "get_robot_state"},
        {"plugin", "vln_agent/GetRobotState"}, {"blocking", false},
        {"settings", {{"rgbd_resource", "rgbd/rgbd"}}}}})}};
  ToolManager manager(host);
  manager.load(config.config(Json::array({navigation, group(8, 0.05)})));
  const auto read = [&] {return payload(invoke(manager, "get_robot_state", Json::object()));};
  const auto unavailable = read();
  ASSERT_TRUE(unavailable.at("ok"));
  EXPECT_EQ(unavailable.at("data").at("camera").at("status"), "unavailable");
  EXPECT_TRUE(unavailable.at("data").at("camera").at("latest_pair_available").is_null());
  createSource();
  Json current;
  ASSERT_TRUE(eventually([&] {
      current = read();
      return current.contains("pose") && current.at("data").at("camera").at("status") == "ready";
    }));
  EXPECT_TRUE(current.at("data").at("camera").at("latest_pair_available"));
  EXPECT_DOUBLE_EQ(current.at("data").at("camera").at("latest_pair_age_sec"), 0.25);
  missing_pair = true;
  current = read();
  EXPECT_EQ(current.at("data").at("camera").at("status"), "no_pair");
  EXPECT_FALSE(current.at("data").at("camera").at("latest_pair_available"));
  EXPECT_TRUE(current.at("data").at("camera").at("latest_pair_age_sec").is_null());
  status_failure = true;
  EXPECT_EQ(read().at("data").at("camera").at("status"), "error");
  status_failure = false;
  status_delay_ms = 150;
  current = read();
  EXPECT_TRUE(current.at("ok"));
  EXPECT_TRUE(current.contains("pose"));
  EXPECT_EQ(current.at("data").at("motion"), "idle");
  EXPECT_EQ(current.at("data").at("camera").at("status"), "timeout");
  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_FALSE(requests.empty());
  for (const auto & request : requests) {EXPECT_EQ(request.first, GetImage::Request::STATUS);}
}

TEST(RgbdMathTest, MaskMeansUseAllValidDepthAndMatchPointProjection)
{
  const auto depth = rgbd::decodeDepth(compressedDepth());
  sensor_msgs::msg::CameraInfo info;
  info.width = 8; info.height = 6;
  info.k = {4, 0, 3, 0, 5, 2, 0, 0, 1};
  info.d = {0.05, 0, 0, 0, 0}; info.distortion_model = "plumb_bob";
  const auto rays = rgbd::projectionRays(info);
  cv::Mat mask(6, 8, CV_8UC1, cv::Scalar(0));
  mask.at<unsigned char>(0, 0) = mask.at<unsigned char>(4, 5) =
    mask.at<unsigned char>(5, 7) = 1;
  const auto mean = rgbd::meanMaskedDepth(depth, rays, mask);
  ASSERT_EQ(mean.count, 2u);
  const auto points = rgbd::meanPointDepth(depth, info, {{0, 0}, {5, 4}, {7, 5}}, {8, 6});
  EXPECT_NEAR(cv::norm(points.point - mean.point), 0, 1e-12);
  const auto first = rgbd::backProject(info, 5, 4, 2), last = rgbd::backProject(info, 7, 5, 2);
  EXPECT_NEAR(mean.point[0], (first.x + last.x) / 2, 1e-12);
  EXPECT_NEAR(mean.point[1], (first.y + last.y) / 2, 1e-12);
  EXPECT_DOUBLE_EQ(mean.point[2], 2);
  mask.setTo(0);
  EXPECT_EQ(rgbd::meanMaskedDepth(depth, rays, mask).count, 0u);
  EXPECT_EQ(rgbd::meanPointDepth(depth, info, {}, {8, 6}).count, 0u);
  auto changed = info; changed.d[0] += 0.01;
  EXPECT_TRUE(rgbd::sameCalibration(info, info));
  EXPECT_FALSE(rgbd::sameCalibration(info, changed));
}

TEST(RgbdMathTest, MaskMeanProjectsVaryingDepthBeforeAveraging)
{
  auto depth = rgbd::decodeDepth(compressedDepth(true));
  depth.pixels.at<std::uint16_t>(4, 5) = 1010; // 1 m
  depth.pixels.at<std::uint16_t>(5, 7) = 260;  // 4 m
  sensor_msgs::msg::CameraInfo info;
  info.width = 8; info.height = 6;
  info.k = {4, 0, 3, 0, 5, 2, 0, 0, 1};
  cv::Mat mask(6, 8, CV_8UC1, cv::Scalar(0));
  mask.at<unsigned char>(4, 5) = mask.at<unsigned char>(5, 7) = 1;
  const auto mean = rgbd::meanMaskedDepth(depth, rgbd::projectionRays(info), mask);
  ASSERT_EQ(mean.count, 2u);
  EXPECT_DOUBLE_EQ(mean.point[0], 2.25);
  EXPECT_DOUBLE_EQ(mean.point[1], 1.4);
  EXPECT_DOUBLE_EQ(mean.point[2], 2.5);
}

TEST_F(RgbdToolsTest, CapturesModesAndReturnsScaledImageWithOnePublicPairId)
{
  createSource();
  for (const double scale : {1.0, 0.5, 0.4, 1.5}) {
    ToolManager manager(host);
    manager.load(config.config(Json::array({group(8, 0.3, scale)})));
    for (const auto & mode : {"latest", "index", "age"}) {
      const auto handle = capture(manager, mode, std::string(mode) == "latest" ? 0 : 2);
      const auto result = payload(handle);
      const auto & meta = result.at("images").at(0).at("metadata");
      EXPECT_EQ(meta.at("pair_id"), result.at("data").at("pair_id"));
      EXPECT_EQ(meta.at("source_pair_id"), std::string(mode) == "latest" ? 200 : 202);
      EXPECT_EQ(result.at("images").at(0).at("time_stamp"), "1970-01-01T00:00:04.000Z");
      const auto url = handle.result.get().at("output").at(1).at("image_url").get<std::string>();
      const auto decoded = cv::imdecode(utils::decodeBase64(url.substr(url.find(',') + 1)), cv::IMREAD_COLOR);
      EXPECT_EQ(decoded.cols, meta.at("width")); EXPECT_EQ(decoded.rows, meta.at("height"));
      EXPECT_EQ(meta.at("original_width"), 8); EXPECT_EQ(meta.at("original_height"), 6);
      EXPECT_TRUE(query(manager, result.at("data").at("pair_id"), 1, 1).at("ok"));
    }
  }
  std::lock_guard<std::mutex> lock(mutex);
  for (const auto operation : {GetImage::Request::LATEST_PAIR, GetImage::Request::QUEUE_PAIR,
      GetImage::Request::QUEUE_PAIR_AT_AGE})
  {
    EXPECT_TRUE(std::any_of(requests.begin(), requests.end(), [&](const auto & r) {return r.first == operation;}));
  }
}

TEST_F(RgbdToolsTest, ProjectsPerInstanceMeansIgnoresNegativePointsAndKeepsPartialResults)
{
  createSource();
  ToolManager manager(host); manager.load(config.config(Json::array({group()})));
  const auto id = payload(capture(manager)).at("data").at("pair_id");
  auto target = instance(5, 4);
  target["points"].push_back({{"x", 7}, {"y", 5}, {"label", 1}});
  target["points"].push_back({{"x", 1}, {"y", 1}, {"label", 0}});
  target["box"] = {{"x_min", 0}, {"y_min", 0}, {"x_max", 8}, {"y_max", 6}};
  std::size_t count; {std::lock_guard<std::mutex> lock(mutex); count = requests.size();}
  const auto result = project(manager, id, Json::array({target, instance(0, 0), instance(5, 4)}));
  ASSERT_TRUE(result.at("ok")) << result;
  EXPECT_EQ(result.at("data").at("success_count"), 2);
  const auto & rows = result.at("data").at("instances");
  EXPECT_EQ(rows.at(0).at("sample_count"), 2);
  EXPECT_NEAR(rows.at(0).at("x").get<double>(), 9, 1e-9);
  EXPECT_NEAR(rows.at(0).at("y").get<double>(), 21.5, 1e-9);
  EXPECT_NEAR(rows.at(0).at("z").get<double>(), 3, 1e-9);
  EXPECT_EQ(rows.at(1).at("code"), "DEPTH_UNAVAILABLE");
  EXPECT_EQ(rows.at(2).at("index"), 2);
  {std::lock_guard<std::mutex> lock(mutex); EXPECT_EQ(requests.size(), count);}
  const auto negative = project(manager, id, Json::array({instance(5, 4, 0)}));
  EXPECT_FALSE(negative.at("ok")); EXPECT_EQ(negative.at("data").at("success_count"), 0);
}

TEST_F(RgbdToolsTest, UsesRoundedScaleNativeDepthAndFrozenExposurePose)
{
  createSource();
  for (const double scale : {0.5, 0.4}) {
    ToolManager manager(host); manager.load(config.config(Json::array({group(8, 0.3, scale)})));
    const auto id = payload(capture(manager)).at("data").at("pair_id");
    const auto result = query(manager, id, 2, 1);
    ASSERT_TRUE(result.at("ok")) << result;
    const auto & point = result.at("data").at("instances").at(0);
    EXPECT_NEAR(point.at("x").get<double>(), scale == 0.5 ? 9.6 : 9.2, 1e-9);
    EXPECT_NEAR(point.at("y").get<double>(), scale == 0.5 ? 21.0 : 21.5, 1e-9);
  }
  ToolManager manager(host); manager.load(config.config(Json::array({group()})));
  const auto id = payload(capture(manager)).at("data").at("pair_id");
  sendTransforms(100, 200);
  ASSERT_TRUE(eventually([&] {
      const auto r = query(manager, id, 5, 4, "rgbd_test_optical");
      return r.value("ok", false) &&
        std::abs(r.at("data").at("instances").at(0).at("x").get<double>() + 179) < 1e-9;
    }));
  const auto old = query(manager, id).at("data").at("instances").at(0);
  EXPECT_NEAR(old.at("x").get<double>(), 9.2, 1e-9);
  EXPECT_NEAR(old.at("y").get<double>(), 21, 1e-9);
}

TEST_F(RgbdToolsTest, EvictsOldPairsAndSupportsConcurrentPointProjection)
{
  createSource();
  ToolManager manager(host); manager.load(config.config(Json::array({group(2)})));
  const auto old = payload(capture(manager)).at("data").at("pair_id"); capture(manager);
  const auto id = payload(capture(manager)).at("data").at("pair_id");
  EXPECT_EQ(query(manager, old).at("code"), "IMAGE_PAIR_NOT_FOUND");
  const Json args{{"pair_id", id}, {"target_frame", "rgbd_test_map"},
    {"use_segmentation", false}, {"instances", Json::array({instance(5, 4), instance(7, 5)})}};
  std::vector<ToolCallHandle> handles;
  for (int i = 0; i < 4; ++i) {handles.push_back(invoke(manager, "depth_project", args));}
  for (const auto & handle : handles) {
    const auto result = payload(handle);
    ASSERT_TRUE(result.at("ok")); EXPECT_EQ(result.at("data").at("success_count"), 2);
  }
  EXPECT_EQ(query(manager, id, 5, 4, "missing_frame").at("code"), "TF_UNAVAILABLE");
  const auto invalid = query(manager, id, 8, 0);
  EXPECT_EQ(invalid.at("code"), "INVALID_PROMPT");
  EXPECT_TRUE(query(manager, id).at("ok"));
}

TEST_F(RgbdToolsTest, HandlesMissingImagesCalibrationAndDepthWithoutLoadingSam)
{
  ToolManager manager(host); manager.load(config.config(Json::array({group(2, 0.03)})));
  const Json args{{"mode", "latest"}, {"detail", 0}};
  EXPECT_EQ(payload(invoke(manager, "capture_pair", args)).at("code"), "IMAGE_SERVICE_UNAVAILABLE");
  missing_pair = true; createSource(false);
  EXPECT_EQ(payload(invoke(manager, "capture_pair", args)).at("code"), "IMAGE_PAIR_UNAVAILABLE");
  missing_pair = false;
  EXPECT_EQ(payload(invoke(manager, "capture_pair", args)).at("code"), "CAMERA_INFO_UNAVAILABLE");
}

TEST_F(RgbdToolsTest, SegmentsRealImageAndReusesImageFeaturesAcrossCalls)
{
  const auto engines = std::getenv("VLN_SAM_ENGINE_DIR");
  const auto image = std::getenv("VLN_SAM_TEST_IMAGE");
  if (!engines || !image) {GTEST_SKIP() << "通过环境变量显式启用开发板真实 SAM 测试";}
  const auto bgr = cv::imread(image);
  ASSERT_FALSE(bgr.empty()); width = bgr.cols; height = bgr.rows;
  createSource(); cv::imencode(".png", bgr, color); color_format = "png";
  auto rgbd_group = group(8, 0.3, 0.5);
  rgbd_group["settings"]["segmentation"]["warmup_on_initialize"] = true;
  ToolManager manager(host);
  const auto init_start = std::chrono::steady_clock::now();
  manager.load(config.config(Json::array({rgbd_group})));
  const double init_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - init_start).count();
  const auto id = payload(capture(manager)).at("data").at("pair_id");
  auto target = instance(106, 58);
  target["box"] = {{"x_min", 42}, {"y_min", 18}, {"x_max", 170}, {"y_max", 106}};
  auto other = instance(125, 60);
  other["box"] = {{"x_min", 75}, {"y_min", 40}, {"x_max", 150}, {"y_max", 100}};
  const auto targets = Json::array({target, other});
  const auto first_start = std::chrono::steady_clock::now();
  const auto result = project(manager, id, targets, "rgbd_test_map", true);
  ASSERT_TRUE(result.at("ok")) << result;
  ASSERT_EQ(result.at("data").at("success_count"), 2);
  for (const auto & point : result.at("data").at("instances")) {
    EXPECT_GT(point.at("sample_count").get<int>(), 100);
    EXPECT_NEAR(point.at("depth_m").get<double>(), 2, 1e-9);
    EXPECT_NEAR(point.at("z").get<double>(), 3, 1e-9);
    EXPECT_TRUE(point.contains("mask_score"));
  }
  const double first_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - first_start).count();
  const auto warm_start = std::chrono::steady_clock::now();
  for (int i = 0; i < 5; ++i) {
    const auto warm = project(manager, id, targets, "rgbd_test_map", true);
    EXPECT_EQ(warm.at("data").at("instances"), result.at("data").at("instances"));
  }
  const double warm_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - warm_start).count() / 5;
  // 四个重入调用仍共享单个模型，掩码及投影结果保持一致。
  const Json args{{"pair_id", id}, {"target_frame", "rgbd_test_map"},
    {"use_segmentation", true}, {"instances", targets}};
  std::vector<ToolCallHandle> concurrent;
  for (int i = 0; i < 4; ++i) {concurrent.push_back(invoke(manager, "depth_project", args));}
  for (const auto & handle : concurrent) {
    EXPECT_EQ(payload(handle).at("data").at("instances"), result.at("data").at("instances"));
  }
  const auto plain = query(manager, id, 106, 58);
  EXPECT_TRUE(plain.at("ok"));
  std::cout << "SAM_TWO_INSTANCES load_with_warmup_ms=" << init_ms <<
    " first_after_warmup_ms=" << first_ms <<
    " cached_pair_mean_ms=" << warm_ms << std::endl;
}

/** @brief 复用 RGB-D 固件，仅增加接收单点/多点目标的假 Action。 */
class ObjectNavigationTest : public RgbdToolsTest
{
protected:
  using Single = nav2_msgs::action::NavigateToPose;
  using Through = nav2_msgs::action::NavigateThroughPoses;
  using SingleGoal = rclcpp_action::ServerGoalHandle<Single>;
  using ThroughGoal = rclcpp_action::ServerGoalHandle<Through>;

  void createNavigation()
  {
    createSource();
    single_name = "/rgbd_test/object_single_" + std::to_string(getpid());
    through_name = "/rgbd_test/object_through_" + std::to_string(getpid());
    single_server = rclcpp_action::create_server<Single>(source, single_name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Single::Goal>) {
        return reject ? rclcpp_action::GoalResponse::REJECT :
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }, [](std::shared_ptr<SingleGoal>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<SingleGoal> goal) {
        std::size_t index;
        {std::lock_guard<std::mutex> lock(goals_mutex); singles.push_back(goal); index = singles.size();}
        if (auto_finish) {
          if (move_camera_with_turn) {sendTransforms(10 + index * 10, 20);}
          if (fail_turn == int(index)) {goal->abort(std::make_shared<Single::Result>());}
          else {goal->succeed(std::make_shared<Single::Result>());}
        }
      });
    through_server = rclcpp_action::create_server<Through>(source, through_name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Through::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }, [](std::shared_ptr<ThroughGoal>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<ThroughGoal> goal) {
        std::lock_guard<std::mutex> lock(goals_mutex); throughs.push_back(std::move(goal));
      });
    const auto single_ready = rclcpp_action::create_client<Single>(source, single_name);
    const auto through_ready = rclcpp_action::create_client<Through>(source, through_name);
    ASSERT_TRUE(single_ready->wait_for_action_server(3s));
    ASSERT_TRUE(through_ready->wait_for_action_server(3s));
  }
  Json navigationGroup()
  {
    return {{"id", "navigation"}, {"definition", share + "/plugins/tools/navigation/tool.json"},
      {"settings", {{"action_name", single_name}, {"navigate_through_poses_action_name", through_name},
          {"navigate_through_poses_behavior_tree", "/test/objects.xml"},
          {"map_frame", "rgbd_test_map"}, {"base_frame", "rgbd_test_base"},
          {"costmap_timeout_sec", 0.3}}},
      {"functions", Json::array({
          {{"name", "navigate_to_objects"}, {"plugin", "vln_agent/NavigateToObjects"}, {"blocking", false}},
          {{"name", "navigate_to_objects_surround"}, {"plugin", "vln_agent/NavigateToObjectsSurround"}, {"blocking", false}},
          {{"name", "navigate_to_pose_start_navigation"}, {"plugin", "vln_agent/NavigateToPoseStartNavigation"}, {"blocking", false}},
          {{"name", "navigate_to_pose_cancel_navigation"}, {"plugin", "vln_agent/NavigateToPoseCancelNavigation"}, {"blocking", false}},
          {{"name", "navigate_through_poses_cancel_navigation"}, {"plugin", "vln_agent/NavigateThroughPosesCancelNavigation"}, {"blocking", false}}})}};
  }
  Json navigate(ToolManager & manager, const Json & id, const Json & instances, bool segment = false)
  {
    return payload(invoke(manager, "navigate_to_objects", {{"pair_id", id},
        {"use_segmentation", segment}, {"instances", instances}}));
  }
  std::shared_ptr<SingleGoal> single(std::size_t i = 0)
  {
    if (!eventually([&] {
        std::lock_guard<std::mutex> lock(goals_mutex); return singles.size() > i;
      })) {throw std::runtime_error("单点目标未到达假 Action");}
    std::lock_guard<std::mutex> lock(goals_mutex); return singles.at(i);
  }
  std::shared_ptr<ThroughGoal> through(std::size_t i = 0)
  {
    if (!eventually([&] {
        std::lock_guard<std::mutex> lock(goals_mutex); return throughs.size() > i;
      })) {throw std::runtime_error("多点目标未到达假 Action");}
    std::lock_guard<std::mutex> lock(goals_mutex); return throughs.at(i);
  }
  static double yaw(const geometry_msgs::msg::Pose & pose)
  {
    return 2 * std::atan2(pose.orientation.z, pose.orientation.w);
  }

  std::string single_name, through_name;
  rclcpp_action::Server<Single>::SharedPtr single_server;
  rclcpp_action::Server<Through>::SharedPtr through_server;
  std::mutex goals_mutex;
  std::vector<std::shared_ptr<SingleGoal>> singles;
  std::vector<std::shared_ptr<ThroughGoal>> throughs;
  std::atomic_bool reject{false}, auto_finish{false};
  bool move_camera_with_turn{false};
  int fail_turn{0};

  Json lookaroundGroup(double pause = 0)
  {
    auto nav = navigationGroup();
    nav["functions"].push_back({{"name", "lookaround"}, {"plugin", "vln_agent/Lookaround"},
        {"blocking", false}, {"settings", {{"pause_sec", pause}, {"tile_size", 64}, {"gap_size", 8}}}});
    return nav;
  }
  Json surroundNavigate(ToolManager & manager, const Json & id, const Json & instances, bool segment = false)
  {
    return payload(invoke(manager, "navigate_to_objects_surround", {{"lookaround_id", id},
        {"use_segmentation", segment}, {"instances", instances}}));
  }
  Json surroundInstance(const Json & observed, int view)
  {
    const auto & rect = observed.at("data").at("views").at(view).at("image_rect");
    return instance(rect.at("x").get<int>() + 5, rect.at("y").get<int>() + 4);
  }
  Json surroundProject(ToolManager & manager, const Json & id, const Json & instances)
  {
    return payload(invoke(manager, "depth_project_surround", {{"lookaround_id", id},
        {"target_frame", "rgbd_test_map"}, {"use_segmentation", false}, {"instances", instances}}));
  }

};

TEST_F(ObjectNavigationTest, DefaultsToSurfaceAndUsesSingleActionWithSharedCancel)
{
  createNavigation();
  ToolManager manager(host);
  // 先装载 navigation，执行时才查询同组已装载的 RGB-D 资源。
  manager.load(config.config(Json::array({navigationGroup(), group()})));
  const auto id = payload(capture(manager)).at("data").at("pair_id");
  const auto result = navigate(manager, id, Json::array({instance(5, 4)}));
  ASSERT_TRUE(result.at("ok")) << result;
  const auto & data = result.at("data");
  EXPECT_EQ(data.at("action"), "navigate_to_pose");
  EXPECT_EQ(data.at("projection").at("success_count"), 1);
  const auto goal = single(); const auto & pose = goal->get_goal()->pose;
  EXPECT_EQ(pose.header.frame_id, "rgbd_test_map");
  EXPECT_NEAR(pose.pose.position.x, 9.2, 1e-9);
  EXPECT_NEAR(pose.pose.position.y, 21, 1e-9);
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 0);
  EXPECT_NEAR(yaw(pose.pose), std::atan2(14, 3.2), 1e-9);
  // 活动普通导航阻止复合调用；即使要求分割也应直接返回。
  EXPECT_EQ(navigate(manager, id, Json::array({instance(5, 4)}), true).at("code"), "NAVIGATION_BUSY");
  const auto canceled = payload(invoke(manager, "navigate_to_pose_cancel_navigation",
      {{"task_id", data.at("task_id")}}));
  ASSERT_TRUE(canceled.at("ok"));
  ASSERT_TRUE(eventually([&] {return goal->is_canceling();}));
  goal->canceled(std::make_shared<Single::Result>());
}

TEST_F(ObjectNavigationTest, OffsetsOrderedGoalsPointsIntermediateYawForwardAndFacesLastObject)
{
  createNavigation();
  ToolManager manager(host); manager.load(config.config(Json::array({navigationGroup(), group()})));
  const auto id = payload(capture(manager)).at("data").at("pair_id");
  auto a = instance(5, 4), b = instance(7, 5);
  a["stand_off_m"] = 0.6; b["stand_off_m"] = 0.4;
  const double approach_a = std::atan2(14, 3.2);
  const double ax = 9.2 - 0.6 * std::cos(approach_a), ay = 21 - 0.6 * std::sin(approach_a);
  const double approach_b = std::atan2(22 - ay, 8.8 - ax);
  const double bx = 8.8 - 0.4 * std::cos(approach_b), by = 22 - 0.4 * std::sin(approach_b);
  const auto result = navigate(manager, id, Json::array({a, b}));
  ASSERT_TRUE(result.at("ok")) << result;
  EXPECT_EQ(result.at("data").at("action"), "navigate_through_poses");
  const auto goal = through(); const auto & request = *goal->get_goal();
  EXPECT_EQ(request.behavior_tree, "/test/objects.xml");
  ASSERT_EQ(request.poses.size(), 2u);
  EXPECT_NEAR(request.poses[0].pose.position.x, ax, 1e-9);
  EXPECT_NEAR(request.poses[0].pose.position.y, ay, 1e-9);
  EXPECT_NEAR(yaw(request.poses[0].pose), std::atan2(by - ay, bx - ax), 1e-9);
  EXPECT_NEAR(request.poses[1].pose.position.x, bx, 1e-9);
  EXPECT_NEAR(request.poses[1].pose.position.y, by, 1e-9);
  EXPECT_NEAR(yaw(request.poses[1].pose), approach_b, 1e-9);
  EXPECT_EQ(request.poses[0].header.stamp, request.poses[1].header.stamp);
  EXPECT_EQ(payload(invoke(manager, "navigate_to_pose_start_navigation",
      {{"frame_id", "rgbd_test_map"}, {"x", 1}, {"y", 2}, {"yaw", 0}})).at("code"), "NAVIGATION_BUSY");
  const auto canceled = payload(invoke(manager, "navigate_through_poses_cancel_navigation",
      {{"task_id", result.at("data").at("task_id")}}));
  ASSERT_TRUE(canceled.at("ok")); ASSERT_TRUE(eventually([&] {return goal->is_canceling();}));
  goal->canceled(std::make_shared<Through::Result>());
}

TEST_F(ObjectNavigationTest, ZeroOffsetAndCoincidentFinalGoalsKeepApproachYaw)
{
  createNavigation();
  ToolManager manager(host); manager.load(config.config(Json::array({navigationGroup(), group()})));
  const auto id = payload(capture(manager)).at("data").at("pair_id");
  auto a = instance(5, 4), b = instance(7, 5); a["stand_off_m"] = nullptr;
  const auto result = navigate(manager, id, Json::array({a, b, b}));
  ASSERT_TRUE(result.at("ok")) << result;
  const auto goal = through(); const auto & poses = goal->get_goal()->poses;
  ASSERT_EQ(poses.size(), 3u);
  const double direction = std::atan2(1, -0.4);
  EXPECT_NEAR(poses[0].pose.position.x, 9.2, 1e-9);
  EXPECT_NEAR(poses[1].pose.position.x, 8.8, 1e-9);
  for (const auto & pose : poses) {EXPECT_NEAR(yaw(pose.pose), direction, 1e-9);}
  goal->succeed(std::make_shared<Through::Result>());
}

TEST_F(ObjectNavigationTest, ProjectionFailureStartsNoActionAndRetainsEveryInstanceResult)
{
  createNavigation();
  ToolManager manager(host); manager.load(config.config(Json::array({navigationGroup(), group()})));
  const auto id = payload(capture(manager)).at("data").at("pair_id");
  const auto partial = navigate(manager, id, Json::array({instance(5, 4), instance(0, 0)}));
  EXPECT_FALSE(partial.at("ok")); EXPECT_EQ(partial.at("code"), "INSTANCE_PROJECTION_FAILED");
  EXPECT_EQ(partial.at("data").at("instances").size(), 2u);
  EXPECT_EQ(partial.at("data").at("instances").at(1).at("code"), "DEPTH_UNAVAILABLE");
  EXPECT_EQ(navigate(manager, "missing_pair", Json::array({instance(5, 4)})).at("code"), "IMAGE_PAIR_NOT_FOUND");
  {std::lock_guard<std::mutex> lock(goals_mutex); EXPECT_TRUE(singles.empty()); EXPECT_TRUE(throughs.empty());}
  reject = true;
  const auto rejected = navigate(manager, id, Json::array({instance(5, 4)}));
  EXPECT_EQ(rejected.at("code"), "NAVIGATION_REJECTED");
  EXPECT_EQ(rejected.at("data").at("projection").at("success_count"), 1);
  reject = false;
  const auto next = navigate(manager, id, Json::array({instance(5, 4)}));
  ASSERT_TRUE(next.at("ok")) << next;
  single()->succeed(std::make_shared<Single::Result>());
}

TEST_F(ObjectNavigationTest, RealSegmentationFeedsOrderedNavigationWithoutFetchingAnotherFrame)
{
  const auto engines = std::getenv("VLN_SAM_ENGINE_DIR");
  const auto image = std::getenv("VLN_SAM_TEST_IMAGE");
  if (!engines || !image) {GTEST_SKIP() << "开发板真实 SAM 测试由环境变量开启";}
  const auto bgr = cv::imread(image); ASSERT_FALSE(bgr.empty());
  width = bgr.cols; height = bgr.rows; createNavigation();
  cv::imencode(".png", bgr, color); color_format = "png";
  auto rgbd_group = group(8, 0.3, 0.5);
  rgbd_group["settings"]["segmentation"]["warmup_on_initialize"] = true;
  ToolManager manager(host); manager.load(config.config(Json::array({navigationGroup(), rgbd_group})));
  const auto id = payload(capture(manager)).at("data").at("pair_id");
  auto a = instance(106, 58), b = instance(125, 60);
  a["box"] = {{"x_min", 42}, {"y_min", 18}, {"x_max", 170}, {"y_max", 106}};
  b["box"] = {{"x_min", 75}, {"y_min", 40}, {"x_max", 150}, {"y_max", 100}};
  std::size_t reads; {std::lock_guard<std::mutex> lock(mutex); reads = requests.size();}
  // 第一次真实分割直接通过复合工具，验证初始化预热后仍重新编码真实 pair。
  const auto result = navigate(manager, id, Json::array({a, b}), true);
  ASSERT_TRUE(result.at("ok")) << result;
  const auto projected = project(manager, id, Json::array({a, b}), "rgbd_test_map", true);
  ASSERT_TRUE(projected.at("ok")) << projected;
  EXPECT_EQ(result.at("data").at("projection").at("instances"), projected.at("data").at("instances"));
  const auto goal = through(); ASSERT_EQ(goal->get_goal()->poses.size(), 2u);
  const auto & points = projected.at("data").at("instances");
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto & pose = goal->get_goal()->poses[i].pose;
    EXPECT_NEAR(pose.position.x, points[i].at("x").get<double>(), 1e-9);
    EXPECT_NEAR(pose.position.y, points[i].at("y").get<double>(), 1e-9);
    EXPECT_GT(points[i].at("sample_count").get<int>(), 100);
  }
  {std::lock_guard<std::mutex> lock(mutex); EXPECT_EQ(requests.size(), reads);}
  goal->succeed(std::make_shared<Through::Result>());
}
TEST_F(ObjectNavigationTest, LookaroundReturnsMosaicAndRetainsEightOriginalPairsOutsideFifo)
{
  width = 160; height = 120;
  createNavigation(); auto_finish = true;
  ToolManager manager(host);
  manager.load(config.config(Json::array({lookaroundGroup(0.01), group(1)})));
  capture(manager);  // 等待 CameraInfo、TF 和 Action 发现完成。
  const auto started = std::chrono::steady_clock::now();
  const auto handle = invoke(manager, "lookaround", Json::object());
  const auto result = payload(handle);
  ASSERT_TRUE(result.at("ok")) << result;
  EXPECT_GE(std::chrono::steady_clock::now() - started, 80ms);
  const auto id = result.at("data").at("lookaround_id");
  ASSERT_EQ(result.at("data").at("views").size(), 8u);
  {
    std::lock_guard<std::mutex> lock(goals_mutex);
    ASSERT_EQ(singles.size(), 8u);
    for (std::size_t i = 0; i < 8; ++i) {
      const auto & pose = singles[i]->get_goal()->pose;
      EXPECT_DOUBLE_EQ(pose.pose.position.x, 6);
      EXPECT_DOUBLE_EQ(pose.pose.position.y, 7);
      EXPECT_NEAR(yaw(pose.pose), (i + 1) * M_PI / 4, 1e-9);
    }
  }
  const auto url = handle.result.get().at("output").at(1).at("image_url").get<std::string>();
  const auto jpeg = utils::decodeBase64(url.substr(std::string("data:image/jpeg;base64,").size()));
  const auto mosaic = cv::imdecode(jpeg, cv::IMREAD_COLOR);
  EXPECT_EQ(mosaic.size(), cv::Size(208, 208));
  Json targets = Json::array();
  std::vector<cv::Point> originals;
  const std::array<cv::Point,8> cells{{{1,0},{0,0},{0,1},{0,2},{1,2},{2,2},{2,1},{2,0}}};
  for (std::size_t i = 0; i < 8; ++i) {
    const auto & rect = result.at("data").at("views").at(i).at("image_rect");
    EXPECT_EQ(rect.at("x"), cells[i].x * 72);
    EXPECT_EQ(rect.at("y"), cells[i].y * 72 + 8);
    const int x = rect.at("x").get<int>() + 31, y = rect.at("y").get<int>() + 19;
    targets.push_back(instance(x, y));
    originals.push_back(rgbd::toDepthPixel(31, 19, {64,48}, {160,120}));
  }
  for (int i = 0; i < 3; ++i) {capture(manager);}  // 普通 FIFO 只有一对，不能淘汰环视缓存。
  const auto ordinary_id = payload(capture(manager)).at("data").at("pair_id");
  const auto projected = surroundProject(manager, id, targets);
  ASSERT_TRUE(projected.at("ok")) << projected;
  EXPECT_EQ(projected.at("data").at("success_count"), 8);
  for (std::size_t i = 0; i < 8; ++i) {
    const auto plain = query(manager, ordinary_id, originals[i].x, originals[i].y);
    const auto & row = projected.at("data").at("instances").at(i);
    EXPECT_EQ(row.at("index"), i); EXPECT_EQ(row.at("view_index"), i);
    for (const auto & field : {"x", "y", "z", "depth_m"}) {
      EXPECT_NEAR(row.at(field).get<double>(), plain.at("data").at("instances").at(0).at(field).get<double>(), 1e-9);
    }
  }
  const auto next = payload(invoke(manager, "lookaround", Json::object()));
  ASSERT_TRUE(next.at("ok")) << next;
  EXPECT_NE(next.at("data").at("lookaround_id"), id);
  EXPECT_EQ(surroundProject(manager, id, targets).at("code"), "LOOKAROUND_NOT_FOUND");
}

TEST_F(ObjectNavigationTest, SurroundProjectionRejectsPaddingAndCrossViewPromptsPerInstance)
{
  createNavigation(); auto_finish = true;
  ToolManager manager(host);
  manager.load(config.config(Json::array({lookaroundGroup(), group()})));
  capture(manager);
  const auto observed = payload(invoke(manager, "lookaround", Json::object()));
  ASSERT_TRUE(observed.at("ok")) << observed;
  const auto id = observed.at("data").at("lookaround_id");
  const auto & rect = observed.at("data").at("views").at(0).at("image_rect");
  const int x = rect.at("x"), y = rect.at("y");
  auto valid = instance(x + 5, y + 4);
  auto cross = valid;
  cross["points"].push_back({{"x", 0}, {"y", 0}, {"label", 0}});
  auto bad_box = valid;
  bad_box["box"] = {{"x_min", 0}, {"y_min", 0}, {"x_max", x + 8}, {"y_max", y + 6}};
  const auto result = surroundProject(manager, id, Json::array({instance(100,100), cross, bad_box, valid}));
  ASSERT_TRUE(result.at("ok")) << result;
  EXPECT_EQ(result.at("data").at("success_count"), 1);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(result.at("data").at("instances").at(i).at("code"), "INVALID_PROMPT");
  }
  EXPECT_TRUE(result.at("data").at("instances").at(3).at("ok"));
}

TEST_F(ObjectNavigationTest, FailedLookaroundKeepsPreviousCacheAndReleasesMotionReservation)
{
  createNavigation(); auto_finish = true;
  ToolManager manager(host);
  manager.load(config.config(Json::array({lookaroundGroup(), group()})));
  capture(manager);
  const auto observed = payload(invoke(manager, "lookaround", Json::object()));
  ASSERT_TRUE(observed.at("ok")) << observed;
  fail_turn = 10;  // 下一轮第二次转向失败。
  const auto failed = payload(invoke(manager, "lookaround", Json::object()));
  EXPECT_EQ(failed.at("code"), "LOOKAROUND_INTERRUPTED");
  const auto & rect = observed.at("data").at("views").at(0).at("image_rect");
  EXPECT_TRUE(surroundProject(manager, observed.at("data").at("lookaround_id"),
      Json::array({instance(rect.at("x").get<int>() + 5, rect.at("y").get<int>() + 4)})).at("ok"));
  const auto normal = payload(invoke(manager, "navigate_to_pose_start_navigation",
      {{"frame_id", "rgbd_test_map"}, {"x", 6}, {"y", 7}, {"yaw", 0}}));
  EXPECT_TRUE(normal.at("ok")) << normal;
}

TEST_F(ObjectNavigationTest, LookaroundWaitsForTurnAndShutdownCancelsWithoutPublishingAnImage)
{
  createNavigation();
  ToolManager manager(host);
  auto navigation = lookaroundGroup();
  navigation["functions"].push_back({{"name", "get_robot_state"},
      {"plugin", "vln_agent/GetRobotState"}, {"blocking", false}});
  manager.load(config.config(Json::array({navigation, group()})));
  capture(manager);
  const auto handle = invoke(manager, "lookaround", Json::object());
  const auto goal = single();
  EXPECT_EQ(handle.result.wait_for(20ms), std::future_status::timeout);
  const auto state = payload(invoke(manager, "get_robot_state", Json::object()));
  EXPECT_EQ(state.at("data").at("motion"), "lookaround");
  EXPECT_EQ(state.at("data").at("task").at("action"), "navigate_to_pose");
  const auto blocked = payload(invoke(manager, "navigate_to_pose_start_navigation",
      {{"frame_id", "rgbd_test_map"}, {"x", 6}, {"y", 7}, {"yaw", 0}}));
  EXPECT_EQ(blocked.at("code"), "NAVIGATION_BUSY");
  EXPECT_EQ(payload(invoke(manager, "lookaround", Json::object())).at("code"), "NAVIGATION_BUSY");
  const auto started = std::chrono::steady_clock::now();
  manager.shutdown();
  EXPECT_LT(std::chrono::steady_clock::now() - started, 1s);
  EXPECT_EQ(payload(handle).at("code"), "TOOL_MANAGER_CLOSED");
  EXPECT_EQ(handle.result.get().at("output").size(), 1u);
  ASSERT_TRUE(eventually([&] {return goal->is_canceling();}));
  goal->canceled(std::make_shared<Single::Result>());
}

TEST_F(ObjectNavigationTest, SurroundProjectionUsesEachViewsCaptureTransform)
{
  createNavigation(); auto_finish = true; move_camera_with_turn = true;
  ToolManager manager(host);
  manager.load(config.config(Json::array({lookaroundGroup(0.03), group()})));
  capture(manager);
  const auto observed = payload(invoke(manager, "lookaround", Json::object()));
  ASSERT_TRUE(observed.at("ok")) << observed;
  Json targets = Json::array();
  for (const int view : {0, 7}) {
    const auto & rect = observed.at("data").at("views").at(view).at("image_rect");
    targets.push_back(instance(rect.at("x").get<int>() + 5, rect.at("y").get<int>() + 4));
  }
  const auto projected = surroundProject(manager, observed.at("data").at("lookaround_id"), targets);
  ASSERT_TRUE(projected.at("ok")) << projected;
  EXPECT_NEAR(projected.at("data").at("instances").at(0).at("x").get<double>(), 9.2, 1e-9);
  EXPECT_NEAR(projected.at("data").at("instances").at(1).at("x").get<double>(), 79.2, 1e-9);
}

TEST_F(ObjectNavigationTest, ShutdownInterruptsConfiguredLookaroundPause)
{
  createNavigation(); auto_finish = true;
  ToolManager manager(host);
  manager.load(config.config(Json::array({lookaroundGroup(2), group()})));
  capture(manager);
  const auto handle = invoke(manager, "lookaround", Json::object());
  ASSERT_TRUE(eventually([&] {
      return navigate(manager, "missing", Json::array({instance(5,4)})).value("code", "") == "NAVIGATION_BUSY";
    }));
  const auto started = std::chrono::steady_clock::now();
  manager.shutdown();
  EXPECT_LT(std::chrono::steady_clock::now() - started, 1s);
  EXPECT_EQ(payload(handle).at("code"), "TOOL_MANAGER_CLOSED");
}

TEST_F(ObjectNavigationTest, SurroundObjectNavigationUsesCachedSingleTargetAndSharedCancellation)
{
  createNavigation(); auto_finish = true;
  ToolManager manager(host);
  manager.load(config.config(Json::array({lookaroundGroup(), group(1)})));
  capture(manager);
  const auto observed = payload(invoke(manager, "lookaround", Json::object()));
  ASSERT_TRUE(observed.at("ok")) << observed;
  auto_finish = false;
  const auto id = observed.at("data").at("lookaround_id");
  auto target = surroundInstance(observed, 3); target["stand_off_m"] = nullptr;
  std::size_t reads;
  {std::lock_guard<std::mutex> lock(mutex); reads = requests.size();}
  const auto result = surroundNavigate(manager, id, Json::array({target}));
  ASSERT_TRUE(result.at("ok")) << result;
  const auto & data = result.at("data");
  EXPECT_EQ(data.at("action"), "navigate_to_pose");
  EXPECT_EQ(data.at("projection").at("lookaround_id"), id);
  EXPECT_EQ(data.at("projection").at("instances").at(0).at("view_index"), 3);
  const auto goal = single(8);
  EXPECT_NEAR(goal->get_goal()->pose.pose.position.x, 9.2, 1e-9);
  EXPECT_NEAR(goal->get_goal()->pose.pose.position.y, 21, 1e-9);
  EXPECT_NEAR(yaw(goal->get_goal()->pose.pose), std::atan2(14,3.2), 1e-9);
  EXPECT_EQ(surroundNavigate(manager, "missing", Json::array({target}), true).at("code"), "NAVIGATION_BUSY");
  const auto canceled = payload(invoke(manager, "navigate_to_pose_cancel_navigation",
      {{"task_id", data.at("task_id")}}));
  EXPECT_EQ(canceled.at("data").at("status"), "canceling");
  ASSERT_TRUE(eventually([&] {return goal->is_canceling();}));
  goal->canceled(std::make_shared<Single::Result>());
  {std::lock_guard<std::mutex> lock(mutex); EXPECT_EQ(requests.size(), reads);}
}

TEST_F(ObjectNavigationTest, SurroundObjectNavigationKeepsCrossViewInputOrderAndStandOffPolicy)
{
  createNavigation(); auto_finish = true; move_camera_with_turn = true;
  ToolManager manager(host);
  manager.load(config.config(Json::array({lookaroundGroup(0.03), group()})));
  capture(manager);
  const auto observed = payload(invoke(manager, "lookaround", Json::object()));
  ASSERT_TRUE(observed.at("ok")) << observed;
  auto_finish = false; move_camera_with_turn = false;
  auto first = surroundInstance(observed, 7), last = surroundInstance(observed, 0);
  first["stand_off_m"] = 0.6; last["stand_off_m"] = 0.4;
  const auto result = surroundNavigate(manager, observed.at("data").at("lookaround_id"),
    Json::array({first,last}));
  ASSERT_TRUE(result.at("ok")) << result;
  const auto & data = result.at("data");
  EXPECT_EQ(data.at("action"), "navigate_through_poses");
  const auto & objects = data.at("projection").at("instances");
  EXPECT_EQ(objects.at(0).at("view_index"), 7);
  EXPECT_EQ(objects.at(1).at("view_index"), 0);
  EXPECT_NEAR(objects.at(0).at("x").get<double>(),79.2,1e-9);
  EXPECT_NEAR(objects.at(1).at("x").get<double>(),9.2,1e-9);
  const auto goal = through();
  EXPECT_EQ(goal->get_goal()->behavior_tree,"/test/objects.xml");
  ASSERT_EQ(goal->get_goal()->poses.size(),2u);
  const auto & a = goal->get_goal()->poses[0].pose;
  const auto & b = goal->get_goal()->poses[1].pose;
  EXPECT_NEAR(std::hypot(a.position.x-79.2,a.position.y-21),0.6,1e-9);
  EXPECT_NEAR(std::hypot(b.position.x-9.2,b.position.y-21),0.4,1e-9);
  const double first_approach = std::atan2(14,73.2);
  EXPECT_NEAR(a.position.x,79.2-0.6*std::cos(first_approach),1e-9);
  EXPECT_NEAR(a.position.y,21-0.6*std::sin(first_approach),1e-9);
  EXPECT_NEAR(yaw(a),std::atan2(b.position.y-a.position.y,b.position.x-a.position.x),1e-9);
  EXPECT_NEAR(yaw(b),std::atan2(21-b.position.y,9.2-b.position.x),1e-9);
  const auto canceled = payload(invoke(manager,"navigate_through_poses_cancel_navigation",
      {{"task_id",data.at("task_id")}}));
  EXPECT_EQ(canceled.at("data").at("status"),"canceling");
  ASSERT_TRUE(eventually([&] {return goal->is_canceling();}));
  goal->canceled(std::make_shared<Through::Result>());
}

TEST_F(ObjectNavigationTest, SurroundObjectNavigationDoesNotSubmitPartialOrInvalidGoals)
{
  createNavigation(); auto_finish = true;
  ToolManager manager(host);
  manager.load(config.config(Json::array({lookaroundGroup(), group()})));
  capture(manager);
  const auto observed = payload(invoke(manager,"lookaround",Json::object()));
  ASSERT_TRUE(observed.at("ok")) << observed;
  const auto id = observed.at("data").at("lookaround_id");
  const auto target = surroundInstance(observed,0);
  const auto partial = surroundNavigate(manager,id,Json::array({target,instance(100,100)}));
  EXPECT_EQ(partial.at("code"),"INSTANCE_PROJECTION_FAILED");
  EXPECT_EQ(partial.at("data").at("success_count"),1);
  EXPECT_FALSE(partial.at("data").at("instances").at(1).at("ok").get<bool>());
  EXPECT_EQ(surroundNavigate(manager,"missing",Json::array({target})).at("code"),"LOOKAROUND_NOT_FOUND");
  auto invalid = target; invalid["stand_off_m"]=-1;
  EXPECT_EQ(surroundNavigate(manager,id,Json::array({invalid})).at("code"),"INVALID_ARGUMENTS");
  const auto next = payload(invoke(manager,"lookaround",Json::object()));
  ASSERT_TRUE(next.at("ok")) << next;
  EXPECT_EQ(surroundNavigate(manager,id,Json::array({target})).at("code"),"LOOKAROUND_NOT_FOUND");
  {std::lock_guard<std::mutex> lock(goals_mutex); EXPECT_EQ(singles.size(),16u); EXPECT_TRUE(throughs.empty());}
}

TEST_F(ObjectNavigationTest, RgbdSurroundProjectionCanOwnTheCacheInEitherGroupLoadOrder)
{
  createNavigation(); auto_finish=true;
  for (const bool rgbd_first : {false,true}) {
    auto rgbd = group();
    rgbd["functions"] = Json::array({{{"name","depth_project_surround"},
        {"plugin","vln_agent/DepthProjectSurround"},{"blocking",false}}});
    ToolManager manager(host);
    manager.load(config.config(rgbd_first ? Json::array({rgbd,lookaroundGroup()}) :
      Json::array({lookaroundGroup(),rgbd})));
    Json observed;
    ASSERT_TRUE(eventually([&] {
        observed=payload(invoke(manager,"lookaround",Json::object())); return observed.value("ok",false);
      })) << observed;
    const auto projected=surroundProject(manager,observed.at("data").at("lookaround_id"),
      Json::array({surroundInstance(observed,0)}));
    ASSERT_TRUE(projected.at("ok")) << projected;
    EXPECT_NEAR(projected.at("data").at("instances").at(0).at("x").get<double>(),9.2,1e-9);
  }
}

}  // namespace vln_agent::test
