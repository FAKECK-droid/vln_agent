#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <vln_agent/tools/tool_manager.hpp>

/** @brief 独立下游工程只链接安装导出目标，验证公共头和插件发现，不执行工具。 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("vln_agent_install_consumer");
  int result = 0;
  {
    vln_agent::ToolManager manager(node);
    manager.load(argv[1]);
    const auto tools = manager.getTools();
    std::cout << "installed tools: " << tools.size() << std::endl;
    result = tools.size() == 4 ? 0 : 1;
    manager.shutdown();
  }
  rclcpp::shutdown();
  return result;
}
