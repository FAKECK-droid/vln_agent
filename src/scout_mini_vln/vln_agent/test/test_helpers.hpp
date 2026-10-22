#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <unistd.h>

#include "vln_agent/tools/tool_manager.hpp"

namespace vln_agent::test
{

using namespace std::chrono_literals;

/** @brief 在构建目录保留每次测试配置，便于失败后复现。 */
class ConfigFile
{
public:
  ConfigFile()
  {
    static std::atomic_int counter{0};
    directory = std::filesystem::path(VLN_TEST_OUTPUT_DIR) /
      (std::to_string(getpid()) + "_" + std::to_string(++counter));
    std::filesystem::create_directories(directory);
  }

  void write(const std::string & name, const std::string & text)
  {
    std::ofstream(directory / name) << text;
  }

  std::string config(const Json & groups, std::size_t concurrent = 4, std::size_t pending = 64)
  {
    write("agent.yaml", Json{
        {"model", {{"ignored", true}}},
        {"tool_manager", {{"max_concurrent_calls", concurrent},
            {"max_pending_calls", pending}, {"groups", groups}}}}.dump(2));
    return (directory / "agent.yaml").string();
  }

  std::filesystem::path directory;
};

/** @brief 生成测试原生函数定义。 */
inline Json definition(const std::string & name)
{
  return {{"type", "function"}, {"name", name}, {"description", "测试函数"},
    {"async", true}, {"strict", true},
    {"parameters", {{"type", "object"}, {"properties", Json::object()},
        {"required", Json::array()}, {"additionalProperties", false}}}};
}

/** @brief 构造完整函数调用项。 */
inline Json call(const std::string & name, const std::string & id, const Json & arguments = Json::object())
{
  return {{"type", "function_call"}, {"status", "completed"},
    {"name", name}, {"call_id", id}, {"arguments", arguments.dump()}};
}

/** @brief 构造管理器接受的完整模型响应。 */
inline Json response(const Json & items)
{
  return {{"status", "completed"}, {"output", items}};
}

/** @brief 验证逐调用 API 外壳并取出业务结果。 */
inline Json payload(const ToolCallHandle & handle)
{
  if (handle.result.wait_for(3s) != std::future_status::ready) {
    throw std::runtime_error("工具 future 未在测试期限内完成: " + handle.call_id);
  }
  const auto output = handle.result.get();
  EXPECT_EQ(output.at("type"), "function_call_output");
  EXPECT_EQ(output.at("call_id"), handle.call_id);
  EXPECT_EQ(output.at("output").at(0).at("type"), "input_text");
  EXPECT_FALSE(output.contains("source"));
  return Json::parse(output.at("output").at(0).at("text").get<std::string>());
}

/** @brief 等待 ROS 回调可观测状态，超时后返回 false。 */
template<class Predicate>
bool eventually(Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

}  // namespace vln_agent::test
