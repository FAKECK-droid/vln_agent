#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <set>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <unistd.h>

#include "vln_agent/events/event_manager.hpp"

namespace vln_agent::test
{
using namespace std::chrono_literals;

class EventManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>(
      "event_manager_test", rclcpp::NodeOptions().use_intra_process_comms(true));
  }

  Json event(int value, int slot = 0)
  {
    const Json payload{{"source", "robot"}, {"origin", "/bt/main/InspectDoor"},
      {"time_stamp", "2026-10-28T06:00:00.000Z"}, {"text", "请检查当前画面"},
      {"instruction", {{"function", "capture_image"}}},
      {"data", {{"value", value}, {"slot", slot}}}};
    return {{"role", "user"},
      {"content", Json::array({{{"type", "input_text"}, {"text", payload.dump()}}})}};
  }

  std::string config(const std::string & text)
  {
    static std::atomic_int counter{0};
    const auto directory = std::filesystem::path(VLN_TEST_OUTPUT_DIR) /
      ("events_" + std::to_string(getpid()) + "_" + std::to_string(++counter));
    std::filesystem::create_directories(directory);
    const auto path = directory / "agent.yaml";
    std::ofstream(path) << text;
    return path.string();
  }

  template<class Predicate>
  bool until(rclcpp::Executor & executor, Predicate predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (predicate()) {return true;}
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(EventManagerTest, PreservesNativeEventsImagesAndCommandsAndReleasesDeliveredData)
{
  EventManager events(node);
  auto picture = event(1);
  const Json image{{"type", "input_image"}, {"image_url", "data:image/jpeg;base64,AA=="},
    {"detail", "auto"}};
  picture["content"].push_back(image);
  const auto other = event(2);
  events.addEvents(picture);
  events.addEvents(Json::array({other, picture}));
  EXPECT_EQ(events.getStatus(), (Json{{"state", "unloaded"}, {"pending_count", 3}}));
  const auto taken = events.takeEvents();
  EXPECT_EQ(taken, Json::array({picture, other, picture}));
  EXPECT_EQ(events.getStatus().at("pending_count"), 0);
  EXPECT_EQ(events.takeEvents(), Json::array());
  events.addEvents(other);
  EXPECT_EQ(events.takeEvents(), Json::array({other}));
  EXPECT_EQ(taken.at(0).at("content").at(1), image);
}

TEST_F(EventManagerTest, ConcurrentBatchesStayContiguousDuringConsumption)
{
  EventManager events(node);
  constexpr int writers = 4, batches = 32, width = 3;
  std::atomic_int finished{0};
  std::vector<std::thread> threads;
  for (int writer = 0; writer < writers; ++writer) {
    threads.emplace_back([&, writer] {
      for (int index = 0; index < batches; ++index) {
        const int id = writer * batches + index;
        events.addEvents(Json::array({event(id, 0), event(id, 1), event(id, 2)}));
      }
      ++finished;
    });
  }
  Json received = Json::array();
  while (finished.load() != writers) {
    utils::appendInputs(received, events.takeEvents());
    std::this_thread::yield();
  }
  for (auto & thread : threads) {thread.join();}
  utils::appendInputs(received, events.takeEvents());
  ASSERT_EQ(received.size(), writers * batches * width);
  std::set<int> ids;
  for (std::size_t i = 0; i < received.size(); i += width) {
    int id = -1;
    for (int slot = 0; slot < width; ++slot) {
      const auto payload = Json::parse(
        received.at(i + slot).at("content").at(0).at("text").get<std::string>());
      if (slot == 0) {id = payload.at("data").at("value").get<int>();}
      EXPECT_EQ(payload.at("data").at("value"), id);
      EXPECT_EQ(payload.at("data").at("slot"), slot);
    }
    EXPECT_TRUE(ids.insert(id).second);
  }
  EXPECT_EQ(ids.size(), writers * batches);
  EXPECT_EQ(events.getStatus().at("pending_count"), 0);
}

TEST_F(EventManagerTest, OneRosTopicAcceptsSingleAndBatchEventsAfterMalformedJson)
{
  EventManager events(node);
  events.load(config("event_manager:\n  receive_topic: /event_manager_test/robot_event\n"));
  auto sender = std::make_shared<rclcpp::Node>(
    "event_sender", rclcpp::NodeOptions().use_intra_process_comms(true));
  auto publisher = sender->create_publisher<std_msgs::msg::String>(
    "/event_manager_test/robot_event",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(sender);
  ASSERT_TRUE(until(executor, [&] {return publisher->get_subscription_count() > 0;}));
  const auto one = event(1), two = event(2), three = event(3);
  for (const auto & text : {std::string("{"), one.dump(), Json::array({two, three}).dump()}) {
    std_msgs::msg::String message;
    message.data = text;
    publisher->publish(message);
  }
  ASSERT_TRUE(until(executor, [&] {return events.getStatus().at("pending_count") == 3;}));
  EXPECT_EQ(events.getStatus().at("state"), "ready");
  EXPECT_EQ(events.takeEvents(), Json::array({one, two, three}));
  EXPECT_TRUE(events.takeEvents().empty());
  events.shutdown();
  EXPECT_EQ(events.getStatus(), (Json{{"state", "closed"}, {"pending_count", 0}}));
}

TEST_F(EventManagerTest, DefaultTopicAndShutdownDoNotOwnHostNodeOrRetainEvents)
{
  EventManager events(node);
  events.load(config("{}\n"));
  auto publisher = node->create_publisher<std_msgs::msg::String>("/vln/robot_event", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(until(executor, [&] {return publisher->get_subscription_count() > 0;}));
  events.addEvents(event(1));
  events.shutdown();
  events.shutdown();
  events.addEvents(event(2));
  EXPECT_EQ(events.getStatus(), (Json{{"state", "closed"}, {"pending_count", 0}}));
  EXPECT_EQ(events.takeEvents(), Json::array());
  EXPECT_EQ(node->get_name(), std::string("event_manager_test"));

  auto owner = std::make_shared<rclcpp::Node>("event_temporary_owner");
  const std::weak_ptr<rclcpp::Node> weak = owner;
  EventManager independent(owner);
  independent.load(config("{}\n"));
  owner.reset();
  EXPECT_TRUE(weak.expired());
  independent.shutdown();
}
}  // namespace vln_agent::test
