#include "vln_agent/utils/base64.hpp"

#include <stdexcept>

namespace vln_agent::utils
{
namespace
{
constexpr std::string_view alphabet =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
}  // namespace

std::string encodeBase64(const std::vector<unsigned char> & bytes)
{
  std::string text;
  text.reserve((bytes.size() + 2) / 3 * 4);
  for (std::size_t i = 0; i < bytes.size(); i += 3) {
    const unsigned value = (unsigned(bytes[i]) << 16) |
      (i + 1 < bytes.size() ? unsigned(bytes[i + 1]) << 8 : 0) |
      (i + 2 < bytes.size() ? unsigned(bytes[i + 2]) : 0);
    text += alphabet[(value >> 18) & 63];
    text += alphabet[(value >> 12) & 63];
    text += i + 1 < bytes.size() ? alphabet[(value >> 6) & 63] : '=';
    text += i + 2 < bytes.size() ? alphabet[value & 63] : '=';
  }
  return text;
}

std::vector<unsigned char> decodeBase64(std::string_view text)
{
  if (text.size() % 4 != 0) {
    throw std::invalid_argument("Base64 长度必须为四的倍数");
  }
  std::vector<unsigned char> bytes;
  bytes.reserve(text.size() / 4 * 3);
  for (std::size_t i = 0; i < text.size(); i += 4) {
    unsigned value = 0;
    int padding = 0;
    for (std::size_t j = 0; j < 4; ++j) {
      const auto c = text[i + j];
      const auto digit = alphabet.find(c);
      if (c == '=') {
        if (i + 4 != text.size() || j < 2) {
          throw std::invalid_argument("Base64 填充位置不合法");
        }
        ++padding;
        value <<= 6;
      } else {
        if (digit == std::string_view::npos || padding != 0) {
          throw std::invalid_argument("Base64 字符不合法");
        }
        value = (value << 6) | static_cast<unsigned>(digit);
      }
    }
    if ((padding == 2 && (value & 0xffff) != 0) ||
      (padding == 1 && (value & 0xff) != 0))
    {
      throw std::invalid_argument("Base64 填充位不为零");
    }
    bytes.push_back(static_cast<unsigned char>(value >> 16));
    if (padding < 2) {
      bytes.push_back(static_cast<unsigned char>(value >> 8));
    }
    if (padding == 0) {
      bytes.push_back(static_cast<unsigned char>(value));
    }
  }
  return bytes;
}

}  // namespace vln_agent::utils
