#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "vln_agent/utils/base64.hpp"
#include "vln_agent/utils/json.hpp"
#include "vln_agent/utils/websocket_url.hpp"

namespace vln_agent::test
{

TEST(Utils, Base64MatchesKnownVectorsAndBinaryData)
{
  for (const auto & sample : {
      std::pair<std::string, std::string>{"", ""},
      {"f", "Zg=="}, {"fo", "Zm8="}, {"foo", "Zm9v"},
      {"foob", "Zm9vYg=="}, {"fooba", "Zm9vYmE="}, {"foobar", "Zm9vYmFy"}})
  {
    const std::vector<unsigned char> bytes(sample.first.begin(), sample.first.end());
    EXPECT_EQ(utils::encodeBase64(bytes), sample.second);
    EXPECT_EQ(utils::decodeBase64(sample.second), bytes);
  }
  const std::vector<unsigned char> binary{0, 255, 128, 1};
  EXPECT_EQ(utils::encodeBase64(binary), "AP+AAQ==");
  EXPECT_EQ(utils::decodeBase64("AP+AAQ=="), binary);
  for (const auto & invalid : {"a", "====", "AA=A", "A===", "Zh==", "Zm9=", "AA!A"}) {
    EXPECT_THROW(utils::decodeBase64(invalid), std::invalid_argument);
  }
}

TEST(Utils, JsonArraysAppendAtomicallyAndResultsKeepNativeImages)
{
  Json destination = Json::array({{{"role", "system"}}});
  utils::appendInputs(destination, Json{{"role", "user"}});
  utils::appendInputs(destination, Json::array_t{{{"type", "function_call_output"}}});
  utils::appendInputs(destination, Json::array({{{"type", "compaction"}}}));
  ASSERT_EQ(destination.size(), 4u);
  const auto previous = destination;
  EXPECT_THROW(utils::appendInputs(destination, Json::array({{{"role", "user"}}, 1})),
    std::invalid_argument);
  EXPECT_EQ(destination, previous);
  utils::prependArray(destination, Json::array({{{"type", "reasoning"}}}));
  EXPECT_EQ(destination.at(0).at("type"), "reasoning");
  const Json image{{"type", "input_image"}, {"image_url", "data:image/jpeg;base64,AA=="}};
  const auto result = utils::functionCallOutput("call_1", Json{{"ok", true}}, {image});
  EXPECT_EQ(result.at("call_id"), "call_1");
  EXPECT_EQ(result.at("output").at(0).at("text"), "{\"ok\":true}");
  EXPECT_EQ(result.at("output").at(1), image);
}

TEST(Utils, YamlKeepsQuotedScalarsAndSettingsDefaults)
{
  const auto yaml = YAML::Load("settings: {quoted: '007', value: 7, enabled: true, list: [a, 2]}");
  EXPECT_EQ(utils::settingsOf(yaml),
    (Json{{"quoted", "007"}, {"value", 7}, {"enabled", true}, {"list", {"a", 2}}}));
  EXPECT_EQ(utils::settingsOf(YAML::Load("{}")), Json::object());
  EXPECT_THROW(utils::settingsOf(YAML::Load("settings: []")), std::invalid_argument);
}

TEST(Utils, NormalizesWebSocketSchemesPathsAndIpv6)
{
  const auto plain = utils::parseWebSocketUrl("http://127.0.0.1:8317/v1///");
  EXPECT_FALSE(plain.secure);
  EXPECT_EQ(plain.host, "127.0.0.1");
  EXPECT_EQ(plain.host_header, "127.0.0.1:8317");
  EXPECT_EQ(plain.target, "/v1/responses");
  EXPECT_EQ(plain.port, "8317");
  const auto tls = utils::parseWebSocketUrl("wss://example.com/v1/responses/");
  EXPECT_TRUE(tls.secure);
  EXPECT_EQ(tls.target, "/v1/responses");
  EXPECT_EQ(tls.port, "443");
  const auto ipv6 = utils::parseWebSocketUrl("ws://[::1]:8080");
  EXPECT_EQ(ipv6.host, "::1");
  EXPECT_EQ(ipv6.host_header, "[::1]:8080");
  EXPECT_EQ(ipv6.target, "/responses");
  EXPECT_THROW(utils::parseWebSocketUrl("ftp://example.com"), std::invalid_argument);
  EXPECT_THROW(utils::parseWebSocketUrl("ws://user@host/v1"), std::invalid_argument);
}

}  // namespace vln_agent::test
