#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "vln_agent/tools/tool_result.hpp"
#include "vln_agent/utils/base64.hpp"
#include "vln_agent/utils/time.hpp"
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
  auto tool = ToolResult::success({{"value", 7}});
  tool.payload["images"] = Json::array({{{"metadata", {{"label", "door"}}}}});
  const auto result = utils::functionCallOutput("call_1", tool.payload, {image});
  EXPECT_EQ(result.at("call_id"), "call_1");
  EXPECT_EQ(Json::parse(result.at("output").at(0).at("text").get<std::string>()), tool.payload);
  EXPECT_EQ(result.at("output").at(1), image);
}

TEST(Utils, CommonPayloadKeepsDescriptionsBesideDataAndPoseIsStamped)
{
  auto input = utils::businessPayload("user", {{"route", "hall"}});
  input["instruction"] = "前往门口";
  EXPECT_EQ(input.at("source"), "user");
  EXPECT_EQ(input.at("instruction"), "前往门口");
  EXPECT_EQ(input.at("data"), (Json{{"route", "hall"}}));
  EXPECT_EQ(input.at("time_stamp").get<std::string>().size(), 24u);
  EXPECT_FALSE(input.contains("origin"));
  EXPECT_FALSE(input.contains("event"));
  EXPECT_FALSE(input.contains("text"));
  const auto pose = utils::posePayload("map", 1.2, 2.3, 0.5, "2026-11-07T02:30:00.080Z");
  EXPECT_EQ(pose, (Json{{"frame_id", "map"}, {"x", 1.2}, {"y", 2.3}, {"yaw", 0.5},
    {"time_stamp", "2026-11-07T02:30:00.080Z"}}));
  EXPECT_FALSE(utils::posePayload("odom", 0, 0, 0).contains("time_stamp"));
  const auto success = ToolResult::success({{"task_id", "nav_7"}});
  EXPECT_EQ(success.payload.at("data"), (Json{{"task_id", "nav_7"}}));
  EXPECT_TRUE(success.payload.at("ok"));
  const auto failure = ToolResult::failure("UNAVAILABLE", "服务未就绪");
  EXPECT_FALSE(failure.payload.at("ok"));
  EXPECT_EQ(failure.payload.at("code"), "UNAVAILABLE");
  EXPECT_EQ(failure.payload.at("source"), "robot");
  EXPECT_EQ(failure.payload.at("data"), Json::object());
}

TEST(Utils, RequiresOrderedMetadataForEveryImageAndRejectsWholeInvalidBatches)
{
  const Json image{{"type", "input_image"}, {"image_url", "data:image/png;base64,AA=="}};
  auto payload = utils::businessPayload("user");
  payload["instruction"] = "查看图片";
  EXPECT_THROW(utils::functionCallOutput("c", payload, {image}), std::invalid_argument);
  payload["images"] = Json::array({{{"metadata", {{"label", "first"}}}},
    {{"metadata", {{"score", 0.7}, {"enabled", true}}}}});
  const auto output = utils::functionCallOutput("c", payload, {image, image});
  EXPECT_EQ(Json::parse(output.at("output")[0].at("text").get<std::string>()), payload);
  auto message = Json{{"role", "user"}, {"content", output.at("output")}};
  Json destination = Json::array();
  utils::appendInputs(destination, message);
  EXPECT_EQ(destination, Json::array({message}));
  const auto previous = destination;
  message["content"].erase(message["content"].end() - 1);
  EXPECT_THROW(utils::appendInputs(destination, Json::array({output, message})), std::invalid_argument);
  EXPECT_EQ(destination, previous);
  EXPECT_THROW(utils::appendInputs(destination, message), std::invalid_argument);
  EXPECT_EQ(destination, previous);
  payload["images"] = Json::array({1});
  EXPECT_THROW(utils::functionCallOutput("c", payload, {image}), std::invalid_argument);
  payload["images"] = Json::object();
  EXPECT_THROW(utils::functionCallOutput("c", payload, {}), std::invalid_argument);
  payload.erase("images");
  EXPECT_NO_THROW(utils::functionCallOutput("c", payload, {}));
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

TEST(Utils, UtcTimeHasFixedMillisecondsAcrossEpochAndTimezones)
{
  using namespace std::chrono;
  EXPECT_EQ(utils::formatUtcTime(system_clock::time_point{}), "1970-01-01T00:00:00.000Z");
  EXPECT_EQ(utils::formatUtcTime(system_clock::time_point{} - milliseconds(1)),
    "1969-12-31T23:59:59.999Z");
  EXPECT_EQ(utils::formatUnixSeconds(1792562400.123), "2026-10-21T06:00:00.123Z");
  EXPECT_EQ(utils::utcNow().size(), 24u);
}

}  // namespace vln_agent::test
