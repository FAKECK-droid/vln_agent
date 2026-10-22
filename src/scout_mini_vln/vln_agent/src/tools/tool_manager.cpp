#include "vln_agent/tools/tool_manager.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include <pluginlib/class_loader.hpp>
#include <yaml-cpp/yaml.h>

#include "vln_agent/tools/tool_plugin_base.hpp"

namespace vln_agent
{
using utils::settingsOf;
using Clock = std::chrono::steady_clock;

/** @brief 将工具类型交给通用 JSON 包装函数。 */
static Json wrapResult(const std::string & call_id, const ToolResult & result)
{
  return utils::functionCallOutput(call_id, result.payload, result.images);
}

/** @brief 单个管理器的装载状态、调用去重记录和有界执行队列。 */
struct ToolManager::Impl
{
  struct Entry
  {
    std::shared_ptr<ToolPluginBase> plugin;  ///< 持久的函数插件实例。
    bool blocking;                         ///< 是否阻止调度后续队列项。
  };

  /** @brief 装载事务；销毁时先关闭对象，再释放插件及加载器。 */
  struct Loaded
  {
    std::unique_ptr<pluginlib::ClassLoader<ToolPluginBase>> loader =
      std::make_unique<pluginlib::ClassLoader<ToolPluginBase>>(
      "vln_agent", "vln_agent::ToolPluginBase");
    std::shared_ptr<SharedResources> resources = std::make_shared<SharedResources>();
    std::shared_ptr<std::atomic_bool> stopping = std::make_shared<std::atomic_bool>(false);
    std::unordered_map<std::string, Entry> pool;
    Json tools = Json::array();

    /** @brief 先通知插件退出等待，再关闭共享业务资源。 */
    void stop() noexcept
    {
      if (stopping->exchange(true)) {
        return;
      }
      for (const auto & item : pool) {
        item.second.plugin->shutdown();
      }
      resources->shutdown();
    }

    ~Loaded()
    {
      stop();
      resources.reset();
      pool.clear();
      loader.reset();
    }
  };

  /** @brief promise 只由排队失败路径或取得此项的工作线程完成一次。 */
  struct Call
  {
    ToolCallHandle handle;
    Json arguments;
    std::promise<Json> promise;
    std::uint64_t sequence;                 ///< 按提交顺序分配的队列序号。
    bool blocking{false};                   ///< 已知工具是否暂停后续派发。
    Clock::time_point queued_at{Clock::now()};  ///< 登记时刻，计入提交中的参数解析时间。
    Clock::time_point started_at{}, finished_at{};  ///< 未开始或未完成时为默认值。
  };

  explicit Impl(rclcpp::Node & node_ref) : node(node_ref) {}

  /** @brief 在局部装载完整配置，成功后才启动执行线程并对外开放。 */
  void load(const std::string & path)
  {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex);
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (loaded || closed) {
        throw std::logic_error("ToolManager 只允许一次成功装载，关闭后不能重新装载");
      }
    }
    const auto config = YAML::LoadFile(path)["tool_manager"];
    if (!config.IsMap() || !config["groups"].IsSequence()) {
      throw std::invalid_argument("缺少 tool_manager.groups 数组");
    }
    const auto concurrent = config["max_concurrent_calls"].as<std::size_t>(4);
    const auto pending_limit = config["max_pending_calls"].as<std::size_t>(64);
    if (concurrent == 0 || pending_limit == 0) {
      throw std::invalid_argument("工具并发容量和等待容量必须大于零");
    }

    auto candidate = std::make_unique<Loaded>();
    std::unordered_set<std::string> groups;
    const auto base = std::filesystem::absolute(path).parent_path();
    for (const auto & group : config["groups"]) {
      const auto id = group["id"].as<std::string>();
      if (!groups.insert(id).second) {
        throw std::invalid_argument("工具组 ID 重复: " + id);
      }
      const auto definition_path = base / group["definition"].as<std::string>();
      std::ifstream stream(definition_path);
      if (!stream) {
        throw std::runtime_error("无法读取工具定义: " + definition_path.string());
      }
      const auto definitions = Json::parse(stream);
      if (!definitions.is_array()) {
        throw std::invalid_argument("工具定义必须为 function 数组");
      }
      std::unordered_map<std::string, Json> index;
      for (const auto & definition : definitions) {
        if (definition.at("type") != "function") {
          throw std::invalid_argument("工具定义中只允许 function");
        }
        const auto name = definition.at("name").get<std::string>();
        if (!index.emplace(name, definition).second) {
          throw std::invalid_argument("定义文件中的函数名重复: " + name);
        }
      }
      if (!group["functions"].IsSequence()) {
        throw std::invalid_argument("工具组缺少 functions 数组: " + id);
      }
      const auto group_settings = settingsOf(group);
      for (const auto & function : group["functions"]) {
        const auto name = function["name"].as<std::string>();
        if (index.count(name) == 0 || candidate->pool.count(name) != 0) {
          throw std::invalid_argument("函数映射不存在或全局重名: " + name);
        }
        auto plugin = candidate->loader->createSharedInstance(
          function["plugin"].as<std::string>());
        candidate->pool.emplace(name, Entry{plugin, function["blocking"].as<bool>(true)});
        plugin->initialize(
          ToolContext{node, candidate->resources, id, group_settings, candidate->stopping},
          settingsOf(function));
        candidate->tools.push_back(index.at(name));
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex);
      max_concurrent = concurrent;
      max_pending = pending_limit;
      loaded = std::move(candidate);
    }
    for (std::size_t i = 0; i < max_concurrent; ++i) {
      workers.emplace_back([this] {work();});
    }
  }

  /** @brief 登记逐调用 future，队列错误直接完成对应 promise。 */
  std::vector<ToolCallHandle> submit(const Json & response)
  {
    if (response.at("status") != "completed" || !response.at("output").is_array()) {
      throw std::invalid_argument("submit 只接受 completed 的完整响应");
    }
    std::vector<ToolCallHandle> handles;
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : response.at("output")) {
      if (item.at("type") != "function_call") {
        continue;
      }
      const auto call_id = item.at("call_id").get<std::string>();
      const auto name = item.at("name").get<std::string>();
      const auto existing = calls.find(call_id);
      if (existing != calls.end()) {
        handles.push_back(existing->second->handle);
        continue;
      }
      auto call = std::make_shared<Call>();
      call->handle = {call_id, name, call->promise.get_future().share()};
      call->sequence = next_sequence++;
      calls.emplace(call_id, call);
      handles.push_back(call->handle);
      if (loaded && loaded->pool.count(name) != 0) {
        call->blocking = loaded->pool.at(name).blocking;
      }
      const auto fail = [&](const std::string & code, const std::string & message) {
          complete(call, wrapResult(call_id, ToolResult::failure(code, message)));
        };
      if (closed || !loaded) {
        fail("TOOL_MANAGER_CLOSED", "工具管理器未装载或已经关闭");
      } else if (loaded->pool.count(name) == 0) {
        fail("UNKNOWN_TOOL", "未知函数: " + name);
      } else if (item.value("status", std::string("completed")) != "completed") {
        fail("INVALID_CALL", "函数调用参数尚未生成完成");
      } else {
        try {
          call->arguments = Json::parse(item.at("arguments").get<std::string>());
          if (!call->arguments.is_object()) {
            throw std::invalid_argument("函数参数必须为 JSON 对象");
          }
          if (pending.size() >= max_pending) {
            fail("TOOL_QUEUE_FULL", "工具等待队列已经达到容量");
          } else {
            pending.push_back(call);
          }
        } catch (const std::exception & error) {
          fail("INVALID_ARGUMENTS", error.what());
        }
      }
    }
    condition.notify_all();
    return handles;
  }

  /** @brief 持锁发布终态和结果；只由拒绝、关闭或执行完成路径调用一次。 */
  void complete(const std::shared_ptr<Call> & call, Json output)
  {
    call->finished_at = Clock::now();
    ++completed;
    call->promise.set_value(std::move(output));
  }

  /** @brief 在同一锁和时间基准下复制调用元数据；不读取或消费 future。 */
  Json snapshot(const Call & call, Clock::time_point now) const
  {
    const auto end = call.finished_at == Clock::time_point{} ? now : call.finished_at;
    const bool started = call.started_at != Clock::time_point{};
    const auto wait_end = started ? call.started_at : end;
    return {
      {"call_id", call.handle.call_id}, {"name", call.handle.name},
      {"sequence", call.sequence}, {"blocking", call.blocking},
      {"state", call.finished_at != Clock::time_point{} ? "finished" : started ? "running" : "waiting"},
      {"wait_ms", std::chrono::duration_cast<std::chrono::milliseconds>(
          wait_end - call.queued_at).count()},
      {"run_ms", started ? std::chrono::duration_cast<std::chrono::milliseconds>(
          end - call.started_at).count() : 0}};
  }

  /**
   * @brief 工作线程持锁按 FIFO 取任务，释放锁后执行插件。
   * @note 阻塞项不等待此前调用结束，但完成前禁止任何线程取后续项。
   */
  void work()
  {
    while (true) {
      std::shared_ptr<Call> call;
      Entry entry;
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [this] {return closed || (!pending.empty() && !blocking);});
        if (closed) {
          return;
        }
        call = pending.front();
        pending.pop_front();
        entry = loaded->pool.at(call->handle.name);
        blocking = call->blocking;
        call->started_at = Clock::now();
        running.emplace(call->sequence, call);
      }
      Json output;
      try {
        output = wrapResult(call->handle.call_id, entry.plugin->execute(call->arguments));
      } catch (const Json::exception & error) {
        output = wrapResult(
          call->handle.call_id, ToolResult::failure("INVALID_ARGUMENTS", error.what()));
      } catch (const std::exception & error) {
        output = wrapResult(
          call->handle.call_id, ToolResult::failure("TOOL_EXECUTION_FAILED", error.what()));
      } catch (...) {
        output = wrapResult(
          call->handle.call_id, ToolResult::failure("TOOL_EXECUTION_FAILED", "工具抛出未知异常"));
      }
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (closed) {
          output = wrapResult(
            call->handle.call_id, ToolResult::failure("TOOL_MANAGER_CLOSED", "工具管理器正在关闭"));
        }
        complete(call, std::move(output));
        running.erase(call->sequence);
        if (entry.blocking) {
          blocking = false;
        }
      }
      condition.notify_all();
    }
  }

  /** @brief 不持队列锁关闭插件或等待线程；加载器最后释放。 */
  void shutdown()
  {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex);
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (closed) {
        return;
      }
      closed = true;
      for (const auto & call : pending) {
        complete(call, wrapResult(
            call->handle.call_id,
            ToolResult::failure("TOOL_MANAGER_CLOSED", "调用尚未执行，工具管理器已关闭")));
      }
      pending.clear();
    }
    condition.notify_all();
    if (loaded) {
      loaded->stop();
    }
    for (auto & worker : workers) {
      worker.join();
    }
    workers.clear();
    std::unique_ptr<Loaded> retired;
    {
      std::lock_guard<std::mutex> lock(mutex);
      retired = std::move(loaded);
    }
    retired.reset();
  }

  rclcpp::Node & node;                      ///< 外部 ROS 节点的非拥有引用。
  std::mutex lifecycle_mutex;               ///< 串行化装载与关闭。
  mutable std::mutex mutex;                 ///< 保护队列、去重记录和调度状态。
  std::condition_variable condition;        ///< 通知容量、工作及关闭变化。
  std::unique_ptr<Loaded> loaded;           ///< 成功提交的完整工具集合。
  bool closed{false};                      ///< 关闭后拒绝新调用。
  bool blocking{false};                    ///< 当前是否有未完成的阻塞调用。
  std::size_t max_concurrent{4};            ///< 活动执行上限。
  std::size_t max_pending{64};              ///< FIFO 等待队列上限。
  std::size_t completed{0};                 ///< 已完成 future 的调用数，包含拒绝和关闭。
  std::uint64_t next_sequence{0};           ///< 当前管理器内的提交序号。
  std::unordered_map<std::string, std::shared_ptr<Call>> calls;  ///< call_id 去重记录。
  std::deque<std::shared_ptr<Call>> pending;  ///< 按提交顺序等待工作线程的调用。
  std::map<std::uint64_t, std::shared_ptr<Call>> running;  ///< 按提交序号排列的活动调用。
  std::vector<std::thread> workers;          ///< 固定容量的工作线程池。
};

ToolManager::ToolManager(const std::shared_ptr<rclcpp::Node> & node)
: impl_(std::make_unique<Impl>(*node))
{
}

ToolManager::~ToolManager()
{
  shutdown();
}

void ToolManager::load(const std::string & global_yaml_path)
{
  impl_->load(global_yaml_path);
}

Json ToolManager::getTools() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->loaded ? impl_->loaded->tools : Json::array();
}

Json ToolManager::getStatus() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return {
    {"state", impl_->closed ? (impl_->loaded ? "closing" : "closed") :
        (impl_->loaded ? "ready" : "unloaded")},
    {"max_concurrent_calls", impl_->max_concurrent}, {"max_pending_calls", impl_->max_pending},
    {"waiting_count", impl_->pending.size()}, {"running_count", impl_->running.size()},
    {"completed_count", impl_->completed}, {"call_count", impl_->calls.size()},
    {"blocking", impl_->blocking}};
}

Json ToolManager::getQueueSnapshot() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  const auto now = Clock::now();
  Json waiting = Json::array(), running = Json::array();
  for (const auto & call : impl_->pending) {
    waiting.push_back(impl_->snapshot(*call, now));
  }
  for (const auto & item : impl_->running) {
    running.push_back(impl_->snapshot(*item.second, now));
  }
  return {{"waiting", std::move(waiting)}, {"running", std::move(running)}};
}

Json ToolManager::getCallStatus(const std::string & call_id) const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  const auto call = impl_->calls.find(call_id);
  return call == impl_->calls.end() ? Json() : impl_->snapshot(*call->second, Clock::now());
}

std::vector<ToolCallHandle> ToolManager::submit(const Json & response)
{
  return impl_->submit(response);
}

void ToolManager::shutdown()
{
  impl_->shutdown();
}

}  // namespace vln_agent
