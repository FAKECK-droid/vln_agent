#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace vln_agent::utils
{
/** @brief SIMD 编码为标准 Base64，末尾按需补等号；可并发调用。 */
std::string encodeBase64(const std::vector<unsigned char> & bytes);
/** @brief SIMD 解码标准 Base64；长度、字符、填充或未使用位不合法时抛出 invalid_argument。 */
std::vector<unsigned char> decodeBase64(std::string_view text);
}  // namespace vln_agent::utils
