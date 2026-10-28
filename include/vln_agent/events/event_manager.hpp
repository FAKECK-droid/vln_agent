#pragma once

#include <memory>
#include <string>

#include "vln_agent/utils/json.hpp"

namespace rclcpp {class Node;}

namespace vln_agent
{
/** @brief 独立的机器事件接收与批量交付；只保存尚未交付的原生输入。 */
class EventManager
{
public:
  explicit EventManager(const std::shared_ptr<rclcpp::Node> & node);
  ~EventManager();

  /** @brief 读取 event_manager.receive_topic，缺省订阅 /vln/robot_event。 */
  void load(const std::string & global_yaml_path);
  /** @brief 追加单个原生 Event 或数组，保持整批顺序；关闭后忽略新增事件。 */
  void addEvents(const Json & events);
  /** @brief 非阻塞取走全部待交付事件，无事件返回 []。 */
  Json takeEvents();
  /** @brief 返回 state（unloaded/ready/closed）与 pending_count，不复制事件正文。 */
  Json getStatus() const;
  /** @brief 释放订阅和待交付数据，可重复调用；不关闭宿主节点。 */
  void shutdown();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace vln_agent
