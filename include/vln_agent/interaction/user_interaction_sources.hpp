#pragma once
#include <memory>

namespace vln_agent
{
class SessionManager;
class ToolManager;
class EventManager;

/** @brief 查询依赖均为弱引用，不接管其他组件的生命周期。 */
struct UserInteractionSources
{
  std::weak_ptr<SessionManager> session;
  std::weak_ptr<ToolManager> tools;
  std::weak_ptr<EventManager> events;
};
}  // namespace vln_agent
