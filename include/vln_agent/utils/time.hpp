#pragma once
#include <chrono>
#include <string>

namespace vln_agent::utils
{
/** @brief UTC、固定三位毫秒，例如 2026-10-21T06:00:00.000Z。 */
std::string formatUtcTime(std::chrono::system_clock::time_point value);
std::string utcNow();
/** @brief 仅用于已知为 Unix 秒的外部协议时间，不用于 ROS 仿真时间。 */
std::string formatUnixSeconds(double value);
}  // namespace vln_agent::utils
