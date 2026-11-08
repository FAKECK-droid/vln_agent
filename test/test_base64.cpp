#include "vln_agent/utils/base64.hpp"

#include <array>
#include <atomic>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <openssl/evp.h>

namespace vln_agent::test
{
namespace
{
constexpr std::string_view alphabet =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/** @brief 独立使用 OpenSSL 生成参考编码，不放宽生产解码器的接受规则。 */
std::string referenceEncode(const std::vector<unsigned char> & bytes)
{
  std::string result((bytes.size() + 2) / 3 * 4 + 1, '\0');
  const int written = EVP_EncodeBlock(reinterpret_cast<unsigned char *>(result.data()),
    bytes.data(), static_cast<int>(bytes.size()));
  result.resize(static_cast<std::size_t>(written));
  return result;
}

/** @brief 可复现的二进制数据，覆盖零字节与所有高位字节。 */
std::vector<unsigned char> randomBytes(std::size_t size)
{
  unsigned state = static_cast<unsigned>(size) + 0x12345678u;
  std::vector<unsigned char> bytes(size);
  for (auto & byte : bytes) {
    state ^= state << 13; state ^= state >> 17; state ^= state << 5;
    byte = static_cast<unsigned char>(state);
  }
  return bytes;
}
}  // namespace

// 必须是该独立测试进程的首个用例：并发触发编码/解码的首次 CPU 分派。
TEST(Base64, ConcurrentFirstUseAndSubsequentCalls)
{
  std::atomic_int ready{0};
  std::atomic_bool go{false};
  std::vector<std::future<bool>> workers;
  for (int i = 0; i < 8; ++i) {
    workers.push_back(std::async(std::launch::async, [&, i] {
        const auto bytes = randomBytes(32768 + i);
        const auto expected = referenceEncode(bytes);
        ++ready;
        while (!go.load()) {std::this_thread::yield();}
        for (int iteration = 0; iteration < 32; ++iteration) {
          if (i % 2 == 0 && utils::encodeBase64(bytes) != expected) {return false;}
          if (utils::decodeBase64(expected) != bytes) {return false;}
          if (utils::encodeBase64(bytes) != expected) {return false;}
        }
        return true;
      }));
  }
  while (ready.load() != 8) {std::this_thread::yield();}
  go = true;
  for (auto & worker : workers) {EXPECT_TRUE(worker.get());}
}

TEST(Base64, MatchesIndependentReferenceAcrossSimdAndPaddingBoundaries)
{
  for (std::size_t size = 0; size <= 2048; ++size) {
    const auto bytes = randomBytes(size);
    const auto expected = referenceEncode(bytes);
    EXPECT_EQ(utils::encodeBase64(bytes), expected) << size;
    EXPECT_EQ(utils::decodeBase64(expected), bytes) << size;
  }
  for (const std::size_t size : {65535, 65536, 65537, 1048575, 1048576, 1048577}) {
    const auto bytes = randomBytes(size);
    const auto expected = referenceEncode(bytes);
    EXPECT_EQ(utils::encodeBase64(bytes), expected) << size;
    EXPECT_EQ(utils::decodeBase64(expected), bytes) << size;
  }
}

TEST(Base64, RejectsInvalidBytesAtVectorBoundariesAndInsideBodies)
{
  auto encoded = referenceEncode(randomBytes(1024));
  const std::array<unsigned char, 10> invalid{' ', '\t', '\n', '\r', '-', '_', 0, 128, 255, '='};
  for (std::size_t offset = 0; offset < 256; ++offset) {
    for (const auto byte : invalid) {
      const auto original = encoded[offset];
      encoded[offset] = static_cast<char>(byte);
      EXPECT_THROW(utils::decodeBase64(encoded), std::invalid_argument) << offset;
      encoded[offset] = original;
    }
  }
  for (const auto text : {"A", "AAA", "====", "A===", "=AAA", "AA=A", "AA==AAAA", "AAA=AAAA"}) {
    EXPECT_THROW(utils::decodeBase64(text), std::invalid_argument) << text;
  }
}

TEST(Base64, RejectsEveryNonzeroUnusedPaddingBit)
{
  // 一字节尾部：第二个符号的低四位必须为零。
  for (unsigned value = 0; value < 256; ++value) {
    auto text = referenceEncode({static_cast<unsigned char>(value)});
    const auto digit = alphabet.find(text[1]);
    for (unsigned low = 1; low < 16; ++low) {
      text[1] = alphabet[digit | low];
      EXPECT_THROW(utils::decodeBase64(text), std::invalid_argument);
    }
  }
  // 二字节尾部：第三个符号的低两位必须为零；同时覆盖 SIMD 主体之后的尾部。
  for (unsigned value = 0; value < 256; ++value) {
    auto bytes = randomBytes(128);
    bytes.back() = static_cast<unsigned char>(value);
    auto text = referenceEncode(bytes);
    const auto digit = alphabet.find(text[text.size() - 2]);
    for (unsigned low = 1; low < 4; ++low) {
      text[text.size() - 2] = alphabet[digit | low];
      EXPECT_THROW(utils::decodeBase64(text), std::invalid_argument);
    }
  }
}
}  // namespace vln_agent::test
