#pragma once

#include <cstddef>

#include "vln_agent/utils/json.hpp"

namespace vln_agent::session
{
struct ContextRange
{
  std::size_t first{0};
  std::size_t last{0};
  bool empty() const {return first == last;}
};

/** @brief 只选择系统提示词之后、最近尾部之前的完整历史，保留未闭合工具调用。 */
ContextRange compactRange(const Json & history, std::size_t recent_items);
/** @brief 摘要模型只读取旧历史的文字证据。 */
Json summaryInput(const Json & history, ContextRange range);
/** @brief 验证云端摘要并原子替换指定历史区间。 */
void installSummary(Json & history, ContextRange range, const Json & output);
}  // namespace vln_agent::session
