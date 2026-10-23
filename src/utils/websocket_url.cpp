#include "vln_agent/utils/websocket_url.hpp"

#include <regex>
#include <stdexcept>
#include <string_view>

namespace vln_agent::utils
{

WebSocketUrl parseWebSocketUrl(const std::string & base_url)
{
  std::smatch match;
  const std::regex pattern(R"(^(http|https|ws|wss)://(\[[^\]]+\]|[^/:?#@]+)(:([0-9]+))?(/[^?#]*)?$)");
  if (!std::regex_match(base_url, match, pattern)) {
    throw std::invalid_argument("无效的 Responses WebSocket 基地址");
  }
  WebSocketUrl url;
  url.secure = match[1] == "https" || match[1] == "wss";
  url.host_header = match[2].str() + match[3].str();
  url.host = match[2];
  if (url.host.front() == '[') {
    url.host = url.host.substr(1, url.host.size() - 2);
  }
  url.port = match[4].matched ? match[4].str() : (url.secure ? "443" : "80");
  url.target = match[5];
  while (!url.target.empty() && url.target.back() == '/') {
    url.target.pop_back();
  }
  constexpr std::string_view suffix = "/responses";
  if (url.target.size() < suffix.size() ||
    url.target.compare(url.target.size() - suffix.size(), suffix.size(), suffix) != 0)
  {
    url.target += suffix;
  }
  return url;
}

}  // namespace vln_agent::utils
