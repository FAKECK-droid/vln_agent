#include "vln_agent/tools/ros_callbacks.hpp"

#include <chrono>

namespace vln_agent
{
namespace
{

/** @brief 将专用执行器绑定到宿主节点的上下文。 */
rclcpp::ExecutorOptions optionsFor(rclcpp::Node & node)
{
  rclcpp::ExecutorOptions options;
  options.context = node.get_node_base_interface()->get_context();
  return options;
}

}  // namespace

ToolCallbacks::ToolCallbacks(rclcpp::Node & node)
: context_(node.get_node_base_interface()->get_context()),
  group_(node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false)),
  executor_(optionsFor(node))
{
  executor_.add_callback_group(group_, node.get_node_base_interface());
  thread_ = std::thread([this] {
      while (!closed_.load() && rclcpp::ok(context_)) {
        // 有限等待让刚启动即关闭的场景也能直接 join，无需启动握手或 detach。
        executor_.spin_once(std::chrono::milliseconds(50));
      }
    });
}

ToolCallbacks::~ToolCallbacks()
{
  shutdown();
}

void ToolCallbacks::shutdown() noexcept
{
  if (!closed_.exchange(true)) {
    thread_.join();
  }
}

}  // namespace vln_agent
