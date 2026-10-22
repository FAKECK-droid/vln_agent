#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace vln_agent::utils
{
/** @brief 将字节数组编码为标准 Base64，末尾按需补等号。 */
std::string encodeBase64(const std::vector<unsigned char> & bytes);
/** @brief 解码标准 Base64；长度、字符或填充不合法时抛出 invalid_argument。 */
std::vector<unsigned char> decodeBase64(std::string_view text);
}  // namespace vln_agent::utils
