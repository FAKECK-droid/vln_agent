#pragma once

#include <memory>
#include "vln_agent/utils/json.hpp"

namespace rclcpp {class Node;}

namespace vln_agent
{
/** @brief 用户渠道接收当前状态的完整快照，并返回已包装好的 Responses input[]。 */
class UserInterfaceBase
{
public:
  virtual ~UserInterfaceBase() = default;
  virtual void initialize(const std::shared_ptr<rclcpp::Node> & node,
                          const Json & settings) = 0;
  /** @brief nullptr 表示只收输入；正常返回表示已接收输出，不要求远端确认。 */
  virtual Json exchange(const std::shared_ptr<const Json> & outgoing) = 0;
  virtual void shutdown() noexcept = 0;
};
}  // namespace vln_agent
