#include "test_interface.hpp"
#include <stdexcept>
#include <pluginlib/class_list_macros.hpp>
namespace vln_agent::test
{
std::map<std::string, InterfaceTrace> & interfaceTraces()
{
  static std::map<std::string, InterfaceTrace> traces;
  return traces;
}
void RecordingInterface::initialize(const std::shared_ptr<rclcpp::Node> &, const Json & settings)
{
  name_ = settings.at("name").get<std::string>();
  interfaceTraces()[name_];
  if (settings.value("fail_initialize", false)) {throw std::runtime_error("初始化失败");}
}
Json RecordingInterface::exchange(const std::shared_ptr<const Json> & outgoing)
{
  auto & trace = interfaceTraces().at(name_);
  ++trace.polls;
  if (outgoing && trace.fail_next) {
    trace.fail_next = false;
    throw std::runtime_error("本批尚未接收");
  }
  if (outgoing) {trace.packets.push_back(outgoing);}
  Json inputs = Json::array();
  inputs.swap(trace.inputs);
  return inputs;
}
void RecordingInterface::shutdown() noexcept {++interfaceTraces()[name_].shutdowns;}
}
PLUGINLIB_EXPORT_CLASS(vln_agent::test::RecordingInterface, vln_agent::UserInterfaceBase)
