#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "vln_agent/interaction/user_interface_base.hpp"

namespace vln_agent
{
class RosTopicInterface final : public UserInterfaceBase
{
public:
  ~RosTopicInterface() override {shutdown();}
  void initialize(const rclcpp::Node::SharedPtr & node, const Json & settings) override;
  Json exchange(const std::shared_ptr<const Json> & outgoing) override;
  void shutdown() noexcept override;

private:
  struct Inbox {Json inputs = Json::array();};
  std::shared_ptr<Inbox> inbox_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
}  // namespace vln_agent
