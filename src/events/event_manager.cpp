#include "vln_agent/events/event_manager.hpp"

#include <exception>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

namespace vln_agent
{
struct EventManager::Impl
{
  /** @brief 回调独立持有缓冲，不引用管理器或宿主节点。 */
  struct Buffer
  {
    void addEvents(const Json & events)
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!closed) {utils::appendInputs(pending, events);}
    }

    mutable std::mutex mutex;
    Json pending = Json::array();
    bool loaded{false}, closed{false};
  };

  explicit Impl(const std::shared_ptr<rclcpp::Node> & owner) : node(owner) {}

  std::weak_ptr<rclcpp::Node> node;
  std::shared_ptr<Buffer> buffer = std::make_shared<Buffer>();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
};

EventManager::EventManager(const std::shared_ptr<rclcpp::Node> & node)
: impl_(std::make_unique<Impl>(node)) {}

EventManager::~EventManager() {shutdown();}

void EventManager::load(const std::string & path)
{
  auto node = impl_->node.lock();
  if (!node) {throw std::logic_error("机器事件 ROS 节点已失效");}
  const auto config = YAML::LoadFile(path)["event_manager"];
  const auto topic = config ? config["receive_topic"].as<std::string>("/vln/robot_event") :
    std::string("/vln/robot_event");
  auto buffer = impl_->buffer;
  const auto logger = node->get_logger();
  std::lock_guard<std::mutex> lock(buffer->mutex);
  if (buffer->loaded || buffer->closed) {
    throw std::logic_error("事件管理器已装载或关闭");
  }
  impl_->subscription = node->create_subscription<std_msgs::msg::String>(
    topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [buffer, logger](std_msgs::msg::String::ConstSharedPtr message) {
      try {
        buffer->addEvents(Json::parse(message->data));
      } catch (const std::exception & error) {
        RCLCPP_WARN(logger, "机器事件 JSON 无效: %s", error.what());
      }
    });
  buffer->loaded = true;
}

void EventManager::addEvents(const Json & events)
{
  impl_->buffer->addEvents(events);
}

Json EventManager::takeEvents()
{
  Json events = Json::array();
  std::lock_guard<std::mutex> lock(impl_->buffer->mutex);
  events.swap(impl_->buffer->pending);
  return events;
}

Json EventManager::getStatus() const
{
  const auto & buffer = *impl_->buffer;
  std::lock_guard<std::mutex> lock(buffer.mutex);
  return {{"state", buffer.closed ? "closed" : (buffer.loaded ? "ready" : "unloaded")},
    {"pending_count", buffer.pending.size()}};
}

void EventManager::shutdown()
{
  rclcpp::SubscriptionBase::SharedPtr retired;
  {
    std::lock_guard<std::mutex> lock(impl_->buffer->mutex);
    impl_->buffer->closed = true;
    impl_->buffer->pending = Json::array();
    retired = std::move(impl_->subscription);
  }
  // 在缓冲锁外释放 ROS 订阅；已在途回调持有独立 Buffer，关闭后不会再追加。
}
}  // namespace vln_agent
