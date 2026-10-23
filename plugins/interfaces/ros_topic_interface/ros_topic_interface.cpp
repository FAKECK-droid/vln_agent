#include "ros_topic_interface.hpp"

#include <exception>
#include <stdexcept>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace vln_agent
{
void RosTopicInterface::initialize(const rclcpp::Node::SharedPtr & node, const Json & settings)
{
  if (publisher_) {
    throw std::logic_error("RosTopicInterface 已初始化");
  }
  inbox_ = std::make_shared<Inbox>();
  publisher_ = node->create_publisher<std_msgs::msg::String>(
    settings.value("send_topic", std::string("/vln/user_output")), 10);
  auto inbox = inbox_;
  auto logger = node->get_logger();
  subscription_ = node->create_subscription<std_msgs::msg::String>(
    settings.value("receive_topic", std::string("/vln/user_input")), 10,
    [inbox, logger](std_msgs::msg::String::ConstSharedPtr message) {
      try {
        utils::appendInputs(inbox->inputs, Json::parse(message->data));
      } catch (const std::exception & error) {
        RCLCPP_WARN(logger, "用户输入 JSON 无效: %s", error.what());
      }
    });
}

Json RosTopicInterface::exchange(const std::shared_ptr<const Json> & outgoing)
{
  if (!inbox_) {
    throw std::logic_error("RosTopicInterface 尚未初始化或已关闭");
  }
  if (outgoing && !outgoing->empty()) {
    std_msgs::msg::String message;
    message.data = outgoing->dump();
    publisher_->publish(message);
  }
  Json inputs = Json::array();
  inputs.swap(inbox_->inputs);
  return inputs;
}

void RosTopicInterface::shutdown() noexcept
{
  subscription_.reset();
  publisher_.reset();
  inbox_.reset();
}
}  // namespace vln_agent

PLUGINLIB_EXPORT_CLASS(vln_agent::RosTopicInterface, vln_agent::UserInterfaceBase)
