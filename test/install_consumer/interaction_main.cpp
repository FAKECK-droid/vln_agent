#include <rclcpp/rclcpp.hpp>
#include <vln_agent/interaction/user_interaction_manager.hpp>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("interaction_consumer");
  vln_agent::UserInteractionManager ui(node, {});
  ui.load(argv[1]);
  ui.exchange(nullptr, vln_agent::Json::array(), vln_agent::Json::array());
  ui.shutdown();
  rclcpp::shutdown();
}
