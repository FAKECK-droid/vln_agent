#pragma once

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <type_traits>
#include <unordered_map>

namespace vln_agent
{

/** @brief 可由共享资源注册表统一关闭的业务对象。 */
class SharedResource
{
public:
  virtual ~SharedResource() = default;
  /** @brief 唤醒等待并停止业务回调；允许重复调用。 */
  virtual void shutdown() noexcept = 0;
};

/** @brief 按名称和实际类型保存资源，每个 ToolManager 独立持有一份。 */
class SharedResources
{
public:
  ~SharedResources();

  /**
   * @brief 获取已有资源，或立即执行工厂创建资源。
   * @tparam T 继承 SharedResource 的具体业务类型。
   * @param name 资源在当前管理器内的唯一名称。
   * @param factory 首次访问时执行的工厂，不会被注册表保存。
   * @return 同名、同类型的共享对象。
   * @throws std::invalid_argument 同名资源的类型不一致。
   * @throws std::logic_error 注册表已经关闭。
   * @note 工厂在注册表锁内执行，不应等待 ROS 响应或递归访问注册表。
   */
  template<class T, class Factory>
  std::shared_ptr<T> getOrCreate(const std::string & name, Factory && factory)
  {
    static_assert(std::is_base_of_v<SharedResource, T>);
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      throw std::logic_error("共享资源注册表已经关闭");
    }
    const auto found = entries_.find(name);
    if (found != entries_.end()) {
      if (found->second.type != std::type_index(typeid(T))) {
        throw std::invalid_argument("共享资源类型不一致: " + name);
      }
      return std::static_pointer_cast<T>(found->second.resource);
    }
    auto resource = factory();
    entries_.emplace(name, Entry{typeid(T), resource});
    return resource;
  }

  /** @brief 在注册表锁外关闭所有资源；对象保留到注册表和插件释放引用。 */
  void shutdown() noexcept;

private:
  struct Entry
  {
    std::type_index type;                   ///< 注册时的具体类型。
    std::shared_ptr<SharedResource> resource;  ///< 受管业务对象。
  };
  std::mutex mutex_;                       ///< 保护注册表与关闭状态。
  bool closed_{false};                     ///< 关闭后禁止继续创建资源。
  std::unordered_map<std::string, Entry> entries_;  ///< 名称到资源的映射。
};

}  // namespace vln_agent
