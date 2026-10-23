#include "vln_agent/utils/time.hpp"

#include <cmath>
#include <ctime>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace vln_agent::utils
{
std::string formatUtcTime(std::chrono::system_clock::time_point value)
{
  using namespace std::chrono;
  const auto seconds_part = floor<seconds>(value.time_since_epoch());
  const auto milliseconds_part = duration_cast<milliseconds>(
    value.time_since_epoch() - seconds_part).count();
  const auto raw = static_cast<std::time_t>(seconds_part.count());
  std::tm utc{};
  if (!gmtime_r(&raw, &utc)) {
    throw std::out_of_range("UTC 时间无法格式化");
  }
  std::ostringstream text;
  text.imbue(std::locale::classic());
  text << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S.")
       << std::setfill('0') << std::setw(3) << milliseconds_part << 'Z';
  return text.str();
}

std::string utcNow()
{
  return formatUtcTime(std::chrono::system_clock::now());
}

std::string formatUnixSeconds(double value)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument("Unix 时间必须为有限数值");
  }
  return formatUtcTime(std::chrono::system_clock::time_point(
    std::chrono::duration_cast<std::chrono::system_clock::duration>(
      std::chrono::round<std::chrono::milliseconds>(std::chrono::duration<double>(value)))));
}
}  // namespace vln_agent::utils
