#include "test_tool.hpp"

#include <algorithm>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

namespace vln_agent::test
{

Probe & probe()
{
  static Probe value;
  return value;
}

Token::Token()
{
  static std::atomic_int next_id{0};
  id = ++next_id;
}

void Token::shutdown() noexcept
{
  if (!closed_) {
    closed_ = true;
    std::lock_guard<std::mutex> lock(probe().mutex);
    ++probe().resource_shutdowns;
  }
}

void TestTool::initialize(const ToolContext & context, const Json & settings)
{
  stopping_ = context.stopping;
  token_ = context.resources->getOrCreate<Token>(context.group_id + "/token", [] {
      return std::make_shared<Token>();
    });
  settings_ = settings;
  group_settings_ = context.group_settings;
  if (settings.value("fail_initialize", false)) {
    throw std::runtime_error("测试初始化失败");
  }
}

ToolResult TestTool::execute(const Json & arguments)
{
  auto & state = probe();
  const auto id = arguments.value("id", std::string("call"));
  {
    std::unique_lock<std::mutex> lock(state.mutex);
    state.started.push_back(id);
    state.peak = std::max(state.peak, ++state.active);
    state.condition.notify_all();
    if (arguments.value("wait", false)) {
      state.condition.wait(lock, [&] {
        return stopping_->load() || state.released.count(id) != 0;
      });
    }
    --state.active;
  }
  if (arguments.value("throw", false)) {
    throw std::runtime_error("测试执行异常");
  }
  auto result = ToolResult::success({{"id", id}, {"token_id", token_->id},
    {"settings", settings_}, {"group_settings", group_settings_}});
  result.images = arguments.value("images", std::vector<Json>{});
  if (!result.images.empty()) {
    Json descriptions = Json::array();
    for (std::size_t i = 0; i < result.images.size(); ++i) {
      descriptions.push_back({{"metadata", {{"index", i}}}});
    }
    result.payload["images"] = arguments.value("image_metadata", descriptions);
  }
  if (arguments.contains("time_stamp")) {result.payload["time_stamp"] = arguments.at("time_stamp");}
  return result;
}

void TestTool::shutdown() noexcept
{
  std::lock_guard<std::mutex> lock(probe().mutex);
  probe().condition.notify_all();
}

}  // namespace vln_agent::test

PLUGINLIB_EXPORT_CLASS(vln_agent::test::TestTool, vln_agent::ToolPluginBase)
