#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "vln_agent/interaction/user_interface_base.hpp"

namespace vln_agent::test
{
struct InterfaceTrace
{
  Json inputs = Json::array();
  std::vector<std::shared_ptr<const Json>> packets;
  bool fail_next{false};
  int polls{0}, shutdowns{0};
};
std::map<std::string, InterfaceTrace> & interfaceTraces();

class RecordingInterface : public UserInterfaceBase
{
public:
  void initialize(const std::shared_ptr<rclcpp::Node> &, const Json & settings) override;
  Json exchange(const std::shared_ptr<const Json> & outgoing) override;
  void shutdown() noexcept override;
private:
  std::string name_;
};
}
