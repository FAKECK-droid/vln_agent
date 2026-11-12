#include "test_helpers.hpp"
#include "vln_agent/plugins/tools/rgbd/utils.hpp"
#include "vln_agent/utils/base64.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rgbd_image_queue/srv/get_image.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace vln_agent::test
{
namespace
{
using GetImage = rgbd_image_queue::srv::GetImage;

/** @brief 构造真实 ROS compressedDepth PNG 包，包含 12 字节 ConfigHeader。 */
sensor_msgs::msg::CompressedImage compressedDepth(bool inverse = false)
{
  cv::Mat pixels(6, 8, CV_16UC1, cv::Scalar(inverse ? 510 : 2000));
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
          {"fixed_frame", "rgbd_test_map"}, {"max_pairs", capacity}, {"timeout_sec", timeout}}},
      {"functions", Json::array({
          {{"name", "capture_pair"}, {"plugin", "vln_agent/CapturePair"}, {"blocking", false},
            {"settings", {{"image_scale", scale}}}},
          {{"name", "get_depth_point"}, {"plugin", "vln_agent/GetDepthPoint"}, {"blocking", false}},
          {{"name", "get_depth_points"}, {"plugin", "vln_agent/GetDepthPoints"}, {"blocking", false}}})}};
  }
  void createSource(bool with_calibration = true)
  {
    depth = compressedDepth();
    cv::imencode(".jpg", cv::Mat(6, 8, CV_8UC3, cv::Scalar(20, 80, 180)), color);
    server = source->create_service<GetImage>(service_name,
      [this](GetImage::Request::SharedPtr request, GetImage::Response::SharedPtr reply) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          requests.emplace_back(request->operation, request->detail);
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
        info.width = 8; info.height = 6;
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
  Json query(ToolManager & manager, const Json & id, const std::string & frame = "rgbd_test_map",
    int u = 5, int v = 4)
  {
    return payload(invoke(manager, "get_depth_point",
        {{"image_pair_id", id}, {"pixel_x", u}, {"pixel_y", v}, {"target_frame", frame}}));
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
  std::atomic_bool missing_pair{false};
  int sequence{0};
};

TEST_F(RgbdToolsTest, PairOperationsReturnOnlyOriginalColorAndCacheQueryableDepth)
{
  createSource();
  ToolManager manager(host);
  manager.load(config.config(Json::array({group()})));
  const std::vector<std::pair<std::string, int>> selections{
    {"latest", 0}, {"index", 2}, {"age", 1500}};
  for (const auto & selection : selections) {
    const auto handle = capture(manager, selection.first, selection.second);
    const auto result = payload(handle);
    const auto & output = handle.result.get().at("output");
    ASSERT_EQ(output.size(), 2u);
    EXPECT_EQ(output.at(1).at("image_url"), "data:image/jpeg;base64," + utils::encodeBase64(color));
    EXPECT_EQ(output.at(1).at("detail"), "high");
    const auto & metadata = result.at("images").at(0).at("metadata");
    EXPECT_EQ(metadata.at("width"), 8);
    EXPECT_EQ(metadata.at("height"), 6);
    EXPECT_EQ(metadata.at("pair_id"), 200 + selection.second);
    EXPECT_EQ(metadata.at("image_pair_id"), result.at("data").at("image_pair_id"));
    EXPECT_EQ(result.at("images").at(0).at("time_stamp"), "1970-01-01T00:00:04.000Z");
    EXPECT_TRUE(query(manager, result.at("data").at("image_pair_id")).at("ok"));
  }
  std::lock_guard<std::mutex> lock(mutex);
  for (const auto expected : {
      std::make_pair(GetImage::Request::LATEST_PAIR, 0u),
      std::make_pair(GetImage::Request::QUEUE_PAIR, 2u),
      std::make_pair(GetImage::Request::QUEUE_PAIR_AT_AGE, 1500u)})
  {
    EXPECT_NE(std::find(requests.begin(), requests.end(), expected), requests.end());
  }
}

TEST_F(RgbdToolsTest, ReturnsResizedJpegAndPngAndUsesTheirActualPixelGrid)
{
  createSource();
  const std::vector<std::pair<double, cv::Size>> cases{
    {0.5, {4, 3}}, {0.4, {3, 2}}, {1.5, {12, 9}}};
  for (const auto & item : cases) {
    SCOPED_TRACE(item.first);
    const bool png = item.first == 0.4;
    color_format = png ? "png" : "jpeg";
    cv::imencode(png ? ".png" : ".jpg",
      cv::Mat(6, 8, CV_8UC3, cv::Scalar(20, 80, 180)), color);
    ToolManager manager(host);
    manager.load(config.config(Json::array({group(8, 0.3, item.first)})));
    const auto handle = capture(manager);
    const auto result = payload(handle);
    const auto url = handle.result.get().at("output").at(1).at("image_url").get<std::string>();
    EXPECT_EQ(url.substr(0, url.find(';')), png ? "data:image/png" : "data:image/jpeg");
    const auto bytes = utils::decodeBase64(url.substr(url.find(',') + 1));
    const auto actual = cv::imdecode(bytes, cv::IMREAD_COLOR);
    ASSERT_EQ(actual.size(), item.second);
    const auto & metadata = result.at("images").at(0).at("metadata");
    EXPECT_EQ(metadata.at("width"), actual.cols);
    EXPECT_EQ(metadata.at("height"), actual.rows);
    EXPECT_EQ(metadata.at("original_width"), 8);
    EXPECT_EQ(metadata.at("original_height"), 6);

    // 非整数缩小时，末端像素中心也按实际宽高映射，原始标定继续参与反投影。
    const bool rounded = item.first == 0.4;
    const auto id = result.at("data").at("image_pair_id");
    const auto mapped = query(manager, id, "rgbd_test_map", actual.cols - 1, actual.rows - 1);
    ASSERT_TRUE(mapped.at("ok")) << mapped;
    const auto & data = mapped.at("data");
    EXPECT_EQ(data.at("depth_pixel_x"), rounded ? 6 : 7);
    EXPECT_EQ(data.at("depth_pixel_y"), rounded ? 4 : 5);
    EXPECT_NEAR(data.at("x").get<double>(), rounded ? 9.2 : 8.8, 1e-9);
    EXPECT_NEAR(data.at("y").get<double>(), rounded ? 21.5 : 22.0, 1e-9);
    EXPECT_NEAR(data.at("z").get<double>(), 3.0, 1e-9);
    EXPECT_EQ(query(manager, id, "rgbd_test_map", actual.cols, 0).at("code"), "DEPTH_UNAVAILABLE");
  }
}

TEST_F(RgbdToolsTest, ReadsNativeDepthWithoutAveragingAndKeepsSampledPoseAfterScaling)
{
  createSource();
  cv::Mat pixels(6, 8, CV_16UC1);
  for (int v = 0; v < pixels.rows; ++v) {
    for (int u = 0; u < pixels.cols; ++u) {
      pixels.at<std::uint16_t>(v, u) = 1000 + 100 * u + 10 * v;
    }
  }
  pixels.at<std::uint16_t>(1, 1) = 0;
  std::vector<unsigned char> png;
  cv::imencode(".png", pixels, png);
  depth.data.resize(12 + png.size());
  std::copy(png.begin(), png.end(), depth.data.begin() + 12);
  ToolManager manager(host);
  manager.load(config.config(Json::array({group(8, 0.3, 0.5)})));
  const auto id = payload(capture(manager)).at("data").at("image_pair_id");
  const auto mapped = query(manager, id, "rgbd_test_map", 2, 1);
  ASSERT_TRUE(mapped.at("ok")) << mapped;
  const auto & data = mapped.at("data");
  EXPECT_EQ(data.at("depth_pixel_x"), 5);
  EXPECT_EQ(data.at("depth_pixel_y"), 3);
  EXPECT_DOUBLE_EQ(data.at("depth_m").get<double>(), 1.53);
  EXPECT_NEAR(data.at("x").get<double>(), 9.694, 1e-9);
  EXPECT_NEAR(data.at("y").get<double>(), 20.765, 1e-9);
  EXPECT_NEAR(data.at("z").get<double>(), 2.53, 1e-9);
  EXPECT_EQ(query(manager, id, "rgbd_test_map", 0, 0).at("code"), "DEPTH_UNAVAILABLE");

  sendTransforms(100, 200);
  ASSERT_TRUE(eventually([&] {
      const auto point = query(manager, id, "rgbd_test_optical", 2, 1);
      return point.value("ok", false) &&
             std::abs(point.at("data").at("x").get<double>() + 179.235) < 1e-9;
    }));
  const auto unchanged = query(manager, id, "rgbd_test_map", 2, 1).at("data");
  EXPECT_NEAR(unchanged.at("x").get<double>(), 9.694, 1e-9);
  EXPECT_NEAR(unchanged.at("y").get<double>(), 20.765, 1e-9);
}

TEST_F(RgbdToolsTest, RejectsColorDimensionsThatDoNotMatchCalibration)
{
  createSource();
  cv::imencode(".jpg", cv::Mat(3, 4, CV_8UC3, cv::Scalar(20, 80, 180)), color);
  ToolManager manager(host);
  manager.load(config.config(Json::array({group()})));
  ASSERT_TRUE(eventually([&] {
      return payload(invoke(manager, "capture_pair", {{"mode", "latest"}, {"detail", 0}})).
        value("code", "") == "UNALIGNED_IMAGE_PAIR";
    }));
}

TEST_F(RgbdToolsTest, FreezesExposurePoseAndTransformsPointIntoCurrentTargetFrame)
{
  createSource();
  ToolManager manager(host);
  manager.load(config.config(Json::array({group()})));
  const auto id = payload(capture(manager)).at("data").at("image_pair_id");
  const auto mapped = query(manager, id);
  ASSERT_TRUE(mapped.at("ok"));
  const auto & point = mapped.at("data");
  EXPECT_EQ(point.at("frame_id"), "rgbd_test_map");
  EXPECT_DOUBLE_EQ(point.at("depth_m").get<double>(), 2);
  EXPECT_NEAR(point.at("x").get<double>(), 9.2, 1e-9);
  EXPECT_NEAR(point.at("y").get<double>(), 21, 1e-9);
  EXPECT_NEAR(point.at("z").get<double>(), 3, 1e-9);
  const auto base = query(manager, id, "rgbd_test_base").at("data");
  EXPECT_NEAR(base.at("x").get<double>(), 3.2, 1e-9);
  EXPECT_NEAR(base.at("y").get<double>(), 14, 1e-9);

  sendTransforms(100, 200);  // 模型尚未完成像素选择时，相机发生移动。
  ASSERT_TRUE(eventually([&] {
      const auto current = query(manager, id, "rgbd_test_optical");
      return current.value("ok", false) &&
             std::abs(current.at("data").at("x").get<double>() + 179) < 1e-9;
    }));
  const auto unchanged = query(manager, id).at("data");
  EXPECT_NEAR(unchanged.at("x").get<double>(), 9.2, 1e-9);
  EXPECT_NEAR(unchanged.at("y").get<double>(), 21, 1e-9);
}

TEST_F(RgbdToolsTest, BoundsCacheAndAllowsConcurrentQueriesWithoutConsumingThePair)
{
  createSource();
  ToolManager manager(host);
  manager.load(config.config(Json::array({group(2)})));
  const auto first = payload(capture(manager)).at("data").at("image_pair_id");
  capture(manager);
  const auto latest = payload(capture(manager)).at("data").at("image_pair_id");
  EXPECT_EQ(query(manager, first).at("code"), "IMAGE_PAIR_NOT_FOUND");
  std::vector<ToolCallHandle> handles;
  for (int i = 0; i < 4; ++i) {
    handles.push_back(invoke(manager, "get_depth_point",
        {{"image_pair_id", latest}, {"pixel_x", 5}, {"pixel_y", 4}, {"target_frame", "rgbd_test_map"}}));
  }
  for (const auto & handle : handles) {
    const auto result = payload(handle);
    ASSERT_TRUE(result.at("ok"));
    EXPECT_NEAR(result.at("data").at("x").get<double>(), 9.2, 1e-9);
  }
  EXPECT_EQ(query(manager, latest, "rgbd_test_map", 0, 0).at("code"), "DEPTH_UNAVAILABLE");
  EXPECT_EQ(query(manager, latest, "rgbd_test_map", 8, 0).at("code"), "DEPTH_UNAVAILABLE");
  EXPECT_EQ(query(manager, latest, "missing_target").at("code"), "TF_UNAVAILABLE");
  EXPECT_TRUE(query(manager, latest).at("ok"));
}

TEST_F(RgbdToolsTest, ReturnsClearErrorsForMissingServicePairOrCalibration)
{
  ToolManager manager(host);
  manager.load(config.config(Json::array({group(2, 0.03)})));
  const Json args{{"mode", "latest"}, {"detail", 0}};
  EXPECT_EQ(payload(invoke(manager, "capture_pair", args)).at("code"), "IMAGE_SERVICE_UNAVAILABLE");
  missing_pair = true;
  createSource(false);
  EXPECT_EQ(payload(invoke(manager, "capture_pair", args)).at("code"), "IMAGE_PAIR_UNAVAILABLE");
  missing_pair = false;
  EXPECT_EQ(payload(invoke(manager, "capture_pair", args)).at("code"), "CAMERA_INFO_UNAVAILABLE");
}
TEST_F(RgbdToolsTest, BatchDepthReturnsOrderedPartialResultsWithoutFetchingMoreImages)
{
  createSource();
  ToolManager manager(host);
  manager.load(config.config(Json::array({group()})));
  const auto id = payload(capture(manager)).at("data").at("image_pair_id");
  std::size_t image_reads;
  {std::lock_guard<std::mutex> lock(mutex); image_reads = requests.size();}
  const auto result = payload(invoke(manager, "get_depth_points",
      {{"image_pair_id", id}, {"target_frame", "rgbd_test_map"}, {"pixels", Json::array({
          {{"pixel_x", 5}, {"pixel_y", 4}}, {{"pixel_x", 0}, {"pixel_y", 0}},
          {{"pixel_x", 8}, {"pixel_y", 0}}, {{"pixel_x", 7}, {"pixel_y", 5}}})}}));
  ASSERT_TRUE(result.at("ok")) << result;
  const auto & data = result.at("data");
  EXPECT_EQ(data.at("image_pair_id"), id);
  EXPECT_EQ(data.at("frame_id"), "rgbd_test_map");
  EXPECT_EQ(data.at("success_count"), 2);
  const auto & points = data.at("points");
  ASSERT_EQ(points.size(), 4u);
  EXPECT_EQ(points.at(0).at("pixel_x"), 5);
  EXPECT_TRUE(points.at(0).at("ok"));
  EXPECT_NEAR(points.at(0).at("x").get<double>(), 9.2, 1e-9);
  EXPECT_EQ(points.at(1).at("pixel_x"), 0);
  EXPECT_FALSE(points.at(1).at("ok"));
  EXPECT_EQ(points.at(1).at("code"), "DEPTH_UNAVAILABLE");
  EXPECT_EQ(points.at(2).at("pixel_x"), 8);
  EXPECT_FALSE(points.at(2).at("ok"));
  EXPECT_EQ(points.at(2).at("code"), "DEPTH_UNAVAILABLE");
  EXPECT_EQ(points.at(3).at("pixel_x"), 7);
  EXPECT_TRUE(points.at(3).at("ok"));
  EXPECT_NEAR(points.at(3).at("x").get<double>(), 8.8, 1e-9);
  for (const auto & point : points) {
    EXPECT_FALSE(point.contains("image_pair_id"));
    EXPECT_FALSE(point.contains("frame_id"));
  }
  {std::lock_guard<std::mutex> lock(mutex); EXPECT_EQ(requests.size(), image_reads);}
}

TEST_F(RgbdToolsTest, BatchDepthUsesScaledPixelsAndFrozenPoseWithCurrentTargetFrame)
{
  createSource();
  ToolManager manager(host);
  manager.load(config.config(Json::array({group(8, 0.3, 0.5)})));
  const auto id = payload(capture(manager)).at("data").at("image_pair_id");
  Json arguments{{"image_pair_id", id}, {"target_frame", "rgbd_test_map"},
    {"pixels", Json::array({{{"pixel_x", 2}, {"pixel_y", 1}},
        {{"pixel_x", 3}, {"pixel_y", 2}}})}};
  const auto mapped = payload(invoke(manager, "get_depth_points", arguments));
  ASSERT_TRUE(mapped.at("ok")) << mapped;
  const auto & points = mapped.at("data").at("points");
  EXPECT_EQ(points.at(0).at("depth_pixel_x"), 5);
  EXPECT_EQ(points.at(0).at("depth_pixel_y"), 3);
  EXPECT_NEAR(points.at(0).at("x").get<double>(), 9.6, 1e-9);
  EXPECT_EQ(points.at(1).at("depth_pixel_x"), 7);
  EXPECT_EQ(points.at(1).at("depth_pixel_y"), 5);
  EXPECT_NEAR(points.at(1).at("x").get<double>(), 8.8, 1e-9);
  sendTransforms(100, 200);
  arguments["target_frame"] = "rgbd_test_optical";
  ASSERT_TRUE(eventually([&] {
      const auto current = payload(invoke(manager, "get_depth_points", arguments));
      return current.value("ok", false) &&
             std::abs(current.at("data").at("points").at(0).at("x").get<double>() + 179) < 1e-9;
    }));
  arguments["target_frame"] = "rgbd_test_map";
  const auto unchanged = payload(invoke(manager, "get_depth_points", arguments));
  EXPECT_EQ(unchanged.at("data").at("points"), points);
}

TEST_F(RgbdToolsTest, BatchDepthKeepsAllFailuresAndEnforcesTheBatchSize)
{
  createSource();
  ToolManager manager(host);
  manager.load(config.config(Json::array({group()})));
  const auto id = payload(capture(manager)).at("data").at("image_pair_id");
  Json arguments{{"image_pair_id", id}, {"target_frame", "rgbd_test_map"},
    {"pixels", Json::array({{{"pixel_x", 0}, {"pixel_y", 0}},
        {{"pixel_x", 8}, {"pixel_y", 0}}})}};
  const auto no_depth = payload(invoke(manager, "get_depth_points", arguments));
  EXPECT_FALSE(no_depth.at("ok"));
  EXPECT_EQ(no_depth.at("code"), "DEPTH_UNAVAILABLE");
  EXPECT_EQ(no_depth.at("data").at("success_count"), 0);
  EXPECT_EQ(no_depth.at("data").at("points").size(), 2u);
  arguments["image_pair_id"] = "missing_pair";
  const auto missing = payload(invoke(manager, "get_depth_points", arguments));
  EXPECT_FALSE(missing.at("ok"));
  EXPECT_EQ(missing.at("code"), "IMAGE_PAIR_NOT_FOUND");
  EXPECT_EQ(missing.at("data").at("points").size(), 2u);
  arguments["pixels"] = Json::array();
  EXPECT_EQ(payload(invoke(manager, "get_depth_points", arguments)).at("code"), "INVALID_ARGUMENTS");
  for (int i = 0; i < 65; ++i) {
    arguments["pixels"].push_back({{"pixel_x", 1}, {"pixel_y", 1}});
  }
  EXPECT_EQ(payload(invoke(manager, "get_depth_points", arguments)).at("code"), "INVALID_ARGUMENTS");
}

TEST_F(RgbdToolsTest, BatchDepthAllowsReentrantCallsOnTheSamePair)
{
  createSource();
  ToolManager manager(host);
  manager.load(config.config(Json::array({group()})));
  const auto id = payload(capture(manager)).at("data").at("image_pair_id");
  const Json arguments{{"image_pair_id", id}, {"target_frame", "rgbd_test_map"},
    {"pixels", Json::array({{{"pixel_x", 5}, {"pixel_y", 4}},
        {{"pixel_x", 7}, {"pixel_y", 5}}})}};
  std::vector<ToolCallHandle> handles;
  for (int i = 0; i < 4; ++i) {handles.push_back(invoke(manager, "get_depth_points", arguments));}
  for (const auto & handle : handles) {
    const auto result = payload(handle);
    ASSERT_TRUE(result.at("ok")) << result;
    EXPECT_EQ(result.at("data").at("success_count"), 2);
    EXPECT_NEAR(result.at("data").at("points").at(0).at("x").get<double>(), 9.2, 1e-9);
  }
}

/** @brief 复用合成 pair，加入只计算路径的 Action、footprint 和导航接收端。 */
class ImageNavigationTest : public RgbdToolsTest
{
protected:
  using Plan = nav2_msgs::action::ComputePathToPose;
  using Navigate = nav2_msgs::action::NavigateToPose;
  using PlanGoal = rclcpp_action::ServerGoalHandle<Plan>;
  using NavigateGoal = rclcpp_action::ServerGoalHandle<Navigate>;

  void createNavigation()
  {
    createSource();
    planner_name = "/rgbd_test/planner_" + std::to_string(getpid());
    navigation_name = "/rgbd_test/navigation_" + std::to_string(getpid());
    map_name = "/rgbd_test/map_" + std::to_string(getpid());
    footprint_name = "/rgbd_test/footprint_" + std::to_string(getpid());
    map.header.frame_id = "rgbd_test_map";
    map.metadata.size_x = map.metadata.size_y = 300;
    map.metadata.resolution = 0.1;
    map.metadata.origin.orientation.w = 1;
    map.data.assign(300 * 300, 0);
    // 靠近第二个目标的独立障碍，让路径长度和净空两种权重选出不同目标。
    map.data[200 * 300 + 100] = 254;
    costmap = source->create_service<nav2_msgs::srv::GetCostmap>(map_name,
      [this](nav2_msgs::srv::GetCostmap::Request::SharedPtr,
        nav2_msgs::srv::GetCostmap::Response::SharedPtr reply) {
        ++map_reads; reply->map = map;
      });
    geometry_msgs::msg::TransformStamped base;
    base.header.frame_id = "rgbd_test_map"; base.child_frame_id = "rgbd_nav_base";
    base.transform.translation.x = 9; base.transform.translation.y = 19;
    base.transform.rotation.w = 1;
    broadcaster->sendTransform(base);
    footprint_pub = source->create_publisher<geometry_msgs::msg::PolygonStamped>(footprint_name, 1);
    footprint_timer = source->create_wall_timer(10ms, [this] {
        geometry_msgs::msg::PolygonStamped footprint;
        footprint.header.frame_id = "rgbd_nav_base";
        for (const auto & xy : {cv::Point2f(0.2, 0.15), cv::Point2f(-0.2, 0.15),
            cv::Point2f(-0.2, -0.15), cv::Point2f(0.2, -0.15)})
        {
          geometry_msgs::msg::Point32 point; point.x = xy.x; point.y = xy.y;
          footprint.polygon.points.push_back(point);
        }
        footprint_pub->publish(footprint);
      });
    planner = rclcpp_action::create_server<Plan>(source, planner_name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Plan::Goal>) {
        return reject_plan ? rclcpp_action::GoalResponse::REJECT :
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<PlanGoal>) {
        ++cancellations; return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<PlanGoal> goal) {
        ++plan_reads;
        const int active = ++active_plans;
        peak_plans.store(std::max(peak_plans.load(), active));
        std::lock_guard<std::mutex> lock(plan_mutex);
        pending_plans.push_back(std::move(goal));
      });
    plan_timer = source->create_wall_timer(5ms, [this] {
        std::lock_guard<std::mutex> lock(plan_mutex);
        for (auto it = pending_plans.begin(); it != pending_plans.end();) {
          const auto & goal = *it;
          auto result = std::make_shared<Plan::Result>();
          if (goal->is_canceling()) {goal->canceled(result);}
          else if (withhold_plan) {++it; continue;}
          else if (fail_plan) {goal->abort(result);}
          else {
            result->path.header = goal->get_goal()->goal.header;
            result->path.poses = {goal->get_goal()->start, goal->get_goal()->goal};
            goal->succeed(result);
          }
          --active_plans;
          it = pending_plans.erase(it);
        }
      });
    navigator = rclcpp_action::create_server<Navigate>(source, navigation_name,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<NavigateGoal>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<NavigateGoal> goal) {
        std::lock_guard<std::mutex> lock(navigation_mutex);
        navigated.push_back(std::move(goal));
      });
  }

  Json navigationGroup(double length_weight = 1, double clearance_weight = 1, double timeout = 0.5)
  {
    return {{"id", "navigation"}, {"definition", share + "/plugins/tools/navigation/tool.json"},
      {"settings", {{"action_name", navigation_name}, {"planner_action", planner_name},
          {"global_costmap_service", map_name}, {"footprint_topic", footprint_name},
          {"map_frame", "rgbd_test_map"}, {"base_frame", "rgbd_nav_base"},
          {"costmap_timeout_sec", 0.3}, {"planning_timeout_sec", timeout}}},
      {"functions", Json::array({
          {{"name", "evaluate_navigation_goals"}, {"plugin", "vln_agent/EvaluateNavigationGoals"},
            {"blocking", false}, {"settings", {{"path_length_weight", length_weight},
                {"clearance_weight", clearance_weight}}}},
          {{"name", "navigate_to_image_target"}, {"plugin", "vln_agent/NavigateToImageTarget"},
            {"blocking", true}, {"settings", {{"path_length_weight", length_weight},
                {"clearance_weight", clearance_weight}}}}})}};
  }
  Json imageGoal(int x, int y, double stand_off = 0)
  {
    return {{"pixel_x", x}, {"pixel_y", y}, {"stand_off_m", stand_off}};
  }
  Json evaluate(ToolManager & manager, const Json & id, const Json & goals)
  {
    return payload(invoke(manager, "evaluate_navigation_goals",
        {{"image_pair_id", id}, {"goals", goals}}));
  }
  void waitForFootprint(ToolManager & manager, const Json & id)
  {
    ASSERT_TRUE(eventually([&] {
        const auto result = evaluate(manager, id, Json::array({imageGoal(2, 1)}));
        return result.value("ok", false);
      }));
    map_reads = 0; plan_reads = 0; peak_plans = 0;
  }

  std::string planner_name, navigation_name, map_name, footprint_name;
  nav2_msgs::msg::Costmap map;
  rclcpp::Service<nav2_msgs::srv::GetCostmap>::SharedPtr costmap;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub;
  rclcpp::TimerBase::SharedPtr footprint_timer, plan_timer;
  rclcpp_action::Server<Plan>::SharedPtr planner;
  rclcpp_action::Server<Navigate>::SharedPtr navigator;
  std::mutex plan_mutex, navigation_mutex;
  std::vector<std::shared_ptr<PlanGoal>> pending_plans;
  std::vector<std::shared_ptr<NavigateGoal>> navigated;
  std::atomic_int map_reads{0}, plan_reads{0}, active_plans{0}, peak_plans{0}, cancellations{0};
  std::atomic_bool withhold_plan{false}, fail_plan{false}, reject_plan{false};
};

TEST_F(ImageNavigationTest, ReturnsAllPairGoalsAndSelectsBestAccordingToIndependentWeights)
{
  createNavigation();
  for (const bool prefer_clearance : {false, true}) {
    ToolManager manager(host);
    // 导航插件先载入，执行时才查已载入的 rgbd，避免装载顺序依赖。
    manager.load(config.config(Json::array({
        navigationGroup(prefer_clearance ? 0 : 1, prefer_clearance ? 1 : 0),
        group(8, 0.3, 0.5)})));
    const auto id = payload(capture(manager)).at("data").at("image_pair_id");
    waitForFootprint(manager, id);
    std::size_t image_reads;
    {std::lock_guard<std::mutex> lock(mutex); image_reads = requests.size();}
    const auto result = evaluate(manager, id,
      Json::array({imageGoal(2, 1), imageGoal(1, 1), imageGoal(4, 0)}));
    ASSERT_TRUE(result.at("ok")) << result;
    const auto & data = result.at("data");
    ASSERT_EQ(data.at("goals").size(), 3u);
    EXPECT_TRUE(data.at("goals").at(0).at("reachable"));
    EXPECT_TRUE(data.at("goals").at(1).at("reachable"));
    EXPECT_FALSE(data.at("goals").at(2).at("reachable"));
    EXPECT_EQ(data.at("goals").at(2).at("reason"), "DEPTH_UNAVAILABLE");
    EXPECT_EQ(data.at("best_goal").at("index"), prefer_clearance ? 0 : 1);
    EXPECT_EQ(map_reads, 1);
    EXPECT_EQ(plan_reads, 2);
    EXPECT_EQ(peak_plans, 1);
    {std::lock_guard<std::mutex> lock(mutex); EXPECT_EQ(requests.size(), image_reads);}
  }
}

TEST_F(ImageNavigationTest, StartsBestApproachFacingTargetAndSharesNavigationBusyState)
{
  createNavigation();
  ToolManager manager(host);
  manager.load(config.config(Json::array({navigationGroup(), group(8, 0.3, 0.5)})));
  const auto id = payload(capture(manager)).at("data").at("image_pair_id");
  waitForFootprint(manager, id);
  auto arguments = imageGoal(2, 1, 0.8); arguments["image_pair_id"] = id;
  const auto result = payload(invoke(manager, "navigate_to_image_target", arguments));
  ASSERT_TRUE(result.at("ok")) << result;
  const auto & data = result.at("data");
  EXPECT_EQ(data.at("status"), "running");
  EXPECT_EQ(map_reads, 1);
  EXPECT_LE(plan_reads.load(), 8);
  EXPECT_EQ(peak_plans, 1);
  const auto & target = data.at("target"); const auto & goal = data.at("goal");
  const double dx = target.at("x").get<double>() - goal.at("x").get<double>();
  const double dy = target.at("y").get<double>() - goal.at("y").get<double>();
  EXPECT_NEAR(std::hypot(dx, dy), 0.8, 1e-9);
  EXPECT_NEAR(std::cos(goal.at("yaw").get<double>()), dx / 0.8, 1e-9);
  EXPECT_NEAR(std::sin(goal.at("yaw").get<double>()), dy / 0.8, 1e-9);
  const auto busy = evaluate(manager, id, Json::array({imageGoal(1, 1)}));
  EXPECT_EQ(busy.at("code"), "NAVIGATION_BUSY");
  std::shared_ptr<NavigateGoal> accepted;
  ASSERT_TRUE(eventually([&] {
      std::lock_guard<std::mutex> lock(navigation_mutex);
      if (navigated.empty()) {return false;}
      accepted = navigated.front(); return true;
    }));
  EXPECT_NEAR(accepted->get_goal()->pose.pose.position.x, goal.at("x").get<double>(), 1e-9);
  accepted->succeed(std::make_shared<Navigate::Result>());
  ASSERT_TRUE(eventually([&] {
      return evaluate(manager, id, Json::array({imageGoal(1, 1)})).value("ok", false);
    }));
  EXPECT_TRUE(query(manager, id, "rgbd_test_map", 2, 1).at("ok"));
}

TEST_F(ImageNavigationTest, ReportsNoPathAndTimeoutWithoutMovingAndCanEvaluateAgain)
{
  createNavigation();
  ToolManager manager(host);
  manager.load(config.config(Json::array({navigationGroup(1, 1, 0.1), group(8, 0.3, 0.5)})));
  const auto id = payload(capture(manager)).at("data").at("image_pair_id");
  waitForFootprint(manager, id);
  const auto goals = Json::array({imageGoal(2, 1)});
  fail_plan = true;
  const auto no_path = evaluate(manager, id, goals);
  ASSERT_TRUE(no_path.at("ok"));
  EXPECT_TRUE(no_path.at("data").at("best_goal").is_null());
  EXPECT_EQ(no_path.at("data").at("goals").at(0).at("reason"), "NO_PATH");
  fail_plan = false; withhold_plan = true;
  const auto timeout = evaluate(manager, id, goals);
  EXPECT_EQ(timeout.at("code"), "PLANNING_TIMEOUT");
  EXPECT_TRUE(timeout.at("data").at("goals").at(0).at("reachable").is_null());
  ASSERT_TRUE(eventually([&] {return cancellations.load() > 0 && active_plans.load() == 0;}));
  withhold_plan = false;
  ASSERT_TRUE(eventually([&] {
      const auto recovered = evaluate(manager, id, goals);
      return recovered.value("ok", false) &&
             !recovered.at("data").at("best_goal").is_null();
    }));
  std::lock_guard<std::mutex> lock(navigation_mutex);
  EXPECT_TRUE(navigated.empty());
}

TEST_F(ImageNavigationTest, InvalidDepthAndOccupiedApproachesSkipPlanningAndNeverStartMotion)
{
  createNavigation();
  ToolManager manager(host);
  manager.load(config.config(Json::array({navigationGroup(), group(8, 0.3, 0.5)})));
  const auto id = payload(capture(manager)).at("data").at("image_pair_id");
  waitForFootprint(manager, id);
  auto arguments = imageGoal(4, 0, 0.8); arguments["image_pair_id"] = id;
  const auto invalid = payload(invoke(manager, "navigate_to_image_target", arguments));
  EXPECT_EQ(invalid.at("code"), "NO_REACHABLE_GOAL");
  EXPECT_EQ(invalid.at("data").at("goals").at(0).at("reason"), "DEPTH_UNAVAILABLE");
  EXPECT_EQ(map_reads, 0); EXPECT_EQ(plan_reads, 0);

  std::fill(map.data.begin(), map.data.end(), 254);
  const auto occupied = evaluate(manager, id, Json::array({imageGoal(2, 1), imageGoal(1, 1)}));
  ASSERT_TRUE(occupied.at("ok"));
  EXPECT_TRUE(occupied.at("data").at("best_goal").is_null());
  EXPECT_EQ(occupied.at("data").at("goals").size(), 2u);
  for (const auto & item : occupied.at("data").at("goals")) {
    EXPECT_FALSE(item.at("reachable"));
    EXPECT_EQ(item.at("reason"), "GOAL_OCCUPIED");
  }
  arguments = imageGoal(2, 1, 0.8); arguments["image_pair_id"] = id;
  EXPECT_EQ(payload(invoke(manager, "navigate_to_image_target", arguments)).at("code"),
    "NO_REACHABLE_GOAL");
  EXPECT_EQ(plan_reads, 0);
  std::lock_guard<std::mutex> lock(navigation_mutex);
  EXPECT_TRUE(navigated.empty());
}

TEST_F(ImageNavigationTest, ReentrantEvaluationSerializesSharedPlannerAndKeepsEveryResult)
{
  createNavigation();
  ToolManager manager(host);
  manager.load(config.config(Json::array({navigationGroup(), group(8, 0.3, 0.5)})));
  const auto id = payload(capture(manager)).at("data").at("image_pair_id");
  waitForFootprint(manager, id);
  std::vector<ToolCallHandle> handles;
  for (int i = 0; i < 4; ++i) {
    handles.push_back(invoke(manager, "evaluate_navigation_goals",
        {{"image_pair_id", id}, {"goals", Json::array({imageGoal(2, 1), imageGoal(1, 1)})}}));
  }
  for (const auto & handle : handles) {
    const auto result = payload(handle);
    ASSERT_TRUE(result.at("ok")) << result;
    EXPECT_EQ(result.at("data").at("goals").size(), 2u);
    EXPECT_FALSE(result.at("data").at("best_goal").is_null());
  }
  EXPECT_EQ(map_reads, 4);
  EXPECT_EQ(plan_reads, 8);
  EXPECT_EQ(peak_plans, 1);
}
}  // namespace vln_agent::test
