#include "vln_agent/utils/base64.hpp"

#include <array>
#include <stdexcept>

#include <libbase64.h>

namespace vln_agent::utils
{
namespace
{
constexpr std::string_view alphabet =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/** @brief 尾部只需常量次查表；主体的字符检查由 SIMD 解码器完成。 */
constexpr auto reverseTable()
{
  std::array<unsigned char, 256> table{};
  for (auto & value : table) {value = 255;}
  for (std::size_t i = 0; i < alphabet.size(); ++i) {
    table[static_cast<unsigned char>(alphabet[i])] = static_cast<unsigned char>(i);
  }
  return table;
}
constexpr auto reverse = reverseTable();

/** @brief C 库的 CPU 分派只初始化一次；局部 static 保证首次并发调用安全。 */
void initializeCodec()
{
  static const bool initialized = [] {
      base64_state state;
      base64_stream_encode_init(&state, 0);  // 一次选择会同时设置编码与解码函数。
      return true;
    }();
  (void)initialized;
}

/** @brief 校验最后四个字符及未使用位，保留原先的严格 Base64 接受规则。 */
void validateTail(std::string_view text)
{
  unsigned value = 0;
  int padding = 0;
  for (std::size_t j = 0; j < 4; ++j) {
    const auto c = static_cast<unsigned char>(text[text.size() - 4 + j]);
    if (c == '=') {
      if (j < 2) {throw std::invalid_argument("Base64 填充位置不合法");}
      ++padding;
      value <<= 6;
    } else {
      const auto digit = reverse[c];
      if (digit == 255 || padding != 0) {
        throw std::invalid_argument("Base64 字符不合法");
      }
      value = (value << 6) | digit;
    }
  }
  if ((padding == 2 && (value & 0xffff) != 0) ||
    (padding == 1 && (value & 0xff) != 0))
  {
    throw std::invalid_argument("Base64 填充位不为零");
  }
}
}  // namespace

std::string encodeBase64(const std::vector<unsigned char> & bytes)
{
  if (bytes.empty()) {return {};}
  std::string text((bytes.size() + 2) / 3 * 4, '\0');
  std::size_t written;
  initializeCodec();
  base64_encode(reinterpret_cast<const char *>(bytes.data()), bytes.size(),
    text.data(), &written, 0);
  return text;
}

std::vector<unsigned char> decodeBase64(std::string_view text)
{
  if (text.size() % 4 != 0) {
    throw std::invalid_argument("Base64 长度必须为四的倍数");
  }
  if (text.empty()) {return {};}
  validateTail(text);
  std::vector<unsigned char> bytes(text.size() / 4 * 3);
  std::size_t written;
  initializeCodec();
  if (base64_decode(text.data(), text.size(), reinterpret_cast<char *>(bytes.data()),
      &written, 0) != 1)
  {
    throw std::invalid_argument("Base64 字符或填充位置不合法");
  }
  bytes.resize(written);
  return bytes;
}
}  // namespace vln_agent::utils
