#include <gtest/gtest.h>

#include <thread>
#include <vector>

#include "ws_fixture.hpp"
#include "vln_agent/session/session_manager.hpp"

namespace vln_agent::test
{
namespace
{

Json take(SessionManager & session)
{
  Json result;
  waitFor([&] {result = session.takeResponse(); return !result.is_null();});
  return result;
}

void ready(SessionManager & session)
{
  waitFor([&] {return session.getRequestStatus().at("transport").at("connected").get<bool>();});
}

}  // namespace

TEST(SessionManager, BuildsNativeRequestsWithoutImplicitInitialization)
{
  WsFixture server([](auto & peer, const Json &, std::size_t) {
      peer.send(Json{{"type", "response.completed"}, {"response", completed("r1")}});
    });
  FixtureFiles files;
  SessionManager session(files.sessionConfig(server.url()));
  ready(session);
  session.setTools(Json::array({{{"type", "function"}, {"name", "example"}}}));
  session.request();
  session.addInputs(message("系统规则", "system"));
  session.addInputs(Json::array_t{message("用户"), message("小车状态")});
  const Json tool_output{{"type", "function_call_output"}, {"call_id", "c1"},
    {"output", Json::array({{{"type", "input_text"}, {"text", "{\"ok\":true}"}}})}};
  session.addInputs(Json::array({tool_output}));
  std::this_thread::sleep_for(30ms);
  EXPECT_TRUE(server.requests().empty());
  EXPECT_TRUE(session.getStreamSnapshot().is_null());
  session.request();
  session.request();
  EXPECT_EQ(take(session), completed("r1"));
  ASSERT_EQ(server.requests().size(), 1u);
  const auto request = server.requests().at(0);
  EXPECT_EQ(request.at("type"), "response.create");
  EXPECT_EQ(request.at("model"), "gpt-6-luna");
  EXPECT_EQ(request.at("store"), true);
  EXPECT_EQ(request.at("parallel_tool_calls"), true);
  EXPECT_EQ(request.at("text"), (Json{{"format", {{"type", "text"}}}}));
  EXPECT_EQ(request.at("context_management").at(0).at("compact_threshold"), 8000);
  EXPECT_EQ(request.at("input"),
    Json::array({message("系统规则", "system"), message("用户"), message("小车状态"), tool_output}));
  EXPECT_EQ(request.at("tools").size(), 1u);
  for (const auto & key : {"api_key", "base_url", "max_request_hz", "stream", "background",
      "previous_response_id", "budget_tokens"})
  {
    EXPECT_FALSE(request.contains(key));
  }
  EXPECT_EQ(session.getContextStatus().at("usage"), completed("r1").at("usage"));
}

TEST(SessionManager, BuffersNextInputsAndToolsUntilCompleteResultIsTaken)
{
  WsFixture server([&](auto & peer, const Json &, std::size_t index) {
      const auto id = "r" + std::to_string(index + 1);
      peer.send(Json{{"type", "response.created"}, {"response", {{"id", id}}}});
      if (index == 0) {server.waitRelease(0);}
      peer.send(Json{{"type", "response.completed"}, {"response", completed(id)}});
    });
  FixtureFiles files;
  SessionManager session(files.sessionConfig(server.url()));
  session.setTools(Json::array({{{"name", "old"}}}));
  session.addInputs(message("first"));
  session.request();
  waitFor([&] {return server.requests().size() == 1;});
  session.setTools(Json::array({{{"name", "new"}}}));
  session.addInputs(Json::array_t{message("second"), message("third")});
  session.request();
  session.request();
  EXPECT_EQ(session.getRequestStatus().at("pending_input_count"), 2);
  server.release(0);
  waitFor([&] {return session.getRequestStatus().at("response_ready").get<bool>();});
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(server.requests().size(), 1u);
  EXPECT_EQ(session.getContextStatus().at("latest_response_id"), "r1");
  EXPECT_EQ(session.takeResponse().at("id"), "r1");
  EXPECT_EQ(take(session).at("id"), "r2");
  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 2u);
  EXPECT_EQ(requests[0].at("tools").at(0).at("name"), "old");
  EXPECT_EQ(requests[1].at("tools").at(0).at("name"), "new");
  EXPECT_EQ(requests[1].at("previous_response_id"), "r1");
  EXPECT_EQ(requests[1].at("input"), Json::array({message("second"), message("third")}));
}

TEST(SessionManager, RateLimitWakesWithoutFurtherExternalCalls)
{
  WsFixture server([](auto & peer, const Json &, std::size_t index) {
      peer.send(Json{{"type", "response.completed"},
          {"response", completed("r" + std::to_string(index + 1))}});
    });
  FixtureFiles files;
  SessionManager session(files.sessionConfig(server.url(), {{"max_request_hz", 4.0}}));
  session.addInputs(message("one"));
  session.request();
  take(session);
  session.addInputs(message("two"));
  session.request();
  EXPECT_EQ(session.getRequestStatus().at("state"), "waiting_interval");
  // 只读取服务端观测，不调用任何能唤醒会话调度器的方法。
  waitFor([&] {return server.requests().size() == 2;});
  const auto starts = server.starts();
  EXPECT_GE(std::chrono::duration<double>(starts[1] - starts[0]).count(), 0.20);
  EXPECT_EQ(take(session).at("id"), "r2");
}

TEST(SessionManager, ModelErrorsKeepInputsAndRequireExplicitRetry)
{
  for (const std::string type : {"error", "response.failed", "response.incomplete"}) {
    const Json error{{"code", "model_error"}, {"message", "测试失败"}};
    WsFixture server([&](auto & peer, const Json &, std::size_t index) {
        if (index == 1) {
          server.waitRelease(1);
          if (type == "error") {
            peer.send(Json{{"type", type}, {"error", error}});
          } else {
            peer.send(Json{{"type", type}, {"response",
                {{"id", "r2"}, {"status", type == "response.failed" ? "failed" : "incomplete"}, {"error", error},
                  {"output", Json::array({{{"type", "function_call"}, {"arguments", "{"}}})}}}});
          }
        } else {
          peer.send(Json{{"type", "response.completed"},
              {"response", completed("r" + std::to_string(index + 1))}});
        }
      });
    FixtureFiles files;
    SessionManager session(files.sessionConfig(server.url()));
    session.addInputs(message("init", "system"));
    session.request();
    take(session);
    session.addInputs(message("failed"));
    session.request();
    waitFor([&] {return server.requests().size() == 2;});
    session.addInputs(message("later"));
    session.request();
    server.release(1);
    waitFor([&] {return session.getRequestStatus().at("state") == "error";});
    EXPECT_TRUE(session.takeResponse().is_null());
    EXPECT_EQ(session.getContextStatus().at("latest_response_id"), "r1");
    const auto status = session.getRequestStatus();
    EXPECT_EQ(status.at("pending_input_count"), 1);
    EXPECT_EQ(status.at("current_input_count"), 1);
    EXPECT_FALSE(status.at("send_requested").get<bool>());
    EXPECT_EQ(status.at("last_error").at("event").at("type"), type);
    EXPECT_TRUE(status.at("transport").at("last_error").is_null());
    session.request();
    EXPECT_EQ(take(session).at("id"), "r3");
    const auto retry = server.requests().at(2);
    EXPECT_EQ(retry.at("input"), Json::array({message("failed"), message("later")}));
    EXPECT_EQ(retry.at("previous_response_id"), "r1");
  }
}

TEST(SessionManager, MissingContextRemainsAnErrorWithoutSilentlyStartingNewChain)
{
  WsFixture server([](auto & peer, const Json &, std::size_t index) {
      if (index == 0) {
        peer.send(Json{{"type", "response.completed"}, {"response", completed("r1")}});
      } else {
        peer.send(Json{{"type", "error"},
            {"error", {{"code", "previous_response_not_found"}, {"message", "失效上下文"}}}});
      }
    });
  FixtureFiles files;
  SessionManager session(files.sessionConfig(server.url()));
  session.addInputs(message("one")); session.request(); take(session);
  session.addInputs(message("two")); session.request();
  waitFor([&] {return session.getRequestStatus().at("state") == "error";});
  session.addInputs(message("three"));
  session.request();
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(server.requests().size(), 2u);
  EXPECT_EQ(session.getContextStatus().at("latest_response_id"), "r1");
  EXPECT_EQ(session.getRequestStatus().at("last_error").at("code"), "previous_response_not_found");
}

TEST(SessionManager, ReconnectKeepsConfirmedChainAndDoesNotReplayInterruptedRequest)
{
  WsFixture server([&](auto & peer, const Json &, std::size_t index) {
      if (index == 1) {
        peer.send(Json{{"type", "response.created"}, {"response", {{"id", "r2"}}}});
        server.waitRelease(1);
        peer.drop();
      } else {
        peer.send(Json{{"type", "response.completed"},
            {"response", completed("r" + std::to_string(index + 1))}});
      }
    });
  FixtureFiles files;
  SessionManager session(files.sessionConfig(server.url()));
  session.addInputs(message("init")); session.request(); take(session);
  session.addInputs(message("interrupted")); session.request();
  waitFor([&] {return server.requests().size() == 2;});
  session.addInputs(message("pending")); session.request();
  server.release(1);
  waitFor([&] {
    const auto status = session.getRequestStatus();
    return status.at("request_interrupted").get<bool>() &&
           status.at("transport").at("connected").get<bool>() &&
           status.at("transport").at("connection_id").get<int>() >= 2;
  });
  EXPECT_TRUE(session.getRequestStatus().at("last_error").is_null());
  EXPECT_FALSE(session.getRequestStatus().at("transport").at("last_error").is_null());
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(server.requests().size(), 2u);
  session.request();
  EXPECT_EQ(take(session).at("id"), "r3");
  EXPECT_EQ(server.requests().at(2).at("previous_response_id"), "r1");
  EXPECT_EQ(server.requests().at(2).at("input"),
    Json::array({message("interrupted"), message("pending")}));
}

TEST(SessionManager, TimeoutDiscardsLateEventsAndRetriesPreservedInputs)
{
  WsFixture server([&](auto & peer, const Json &, std::size_t index) {
      if (index == 1) {
        peer.send(Json{{"type", "response.created"}, {"response", {{"id", "late"}}}});
        server.waitRelease(1);
        peer.send(Json{{"type", "response.completed"}, {"response", completed("late")}});
      } else {
        peer.send(Json{{"type", "response.completed"},
            {"response", completed("r" + std::to_string(index + 1))}});
      }
    });
  FixtureFiles files;
  SessionManager session(files.sessionConfig(server.url(), {{"request_timeout_sec", 0.1}}));
  session.addInputs(message("init")); session.request(); take(session);
  session.addInputs(message("timeout")); session.request();
  waitFor([&] {return server.requests().size() == 2;});
  session.addInputs(message("new")); session.request();
  waitFor([&] {return session.getRequestStatus().at("state") == "error";});
  EXPECT_EQ(session.getRequestStatus().at("last_error").at("code"), "REQUEST_TIMEOUT");
  server.release(1);
  ready(session);
  EXPECT_EQ(session.getContextStatus().at("latest_response_id"), "r1");
  EXPECT_TRUE(session.takeResponse().is_null());
  EXPECT_EQ(server.requests().size(), 2u);
  session.request();
  EXPECT_EQ(take(session).at("id"), "r3");
  EXPECT_EQ(server.requests().at(2).at("input"), Json::array({message("timeout"), message("new")}));
  EXPECT_EQ(server.requests().at(2).at("previous_response_id"), "r1");
}

TEST(SessionManager, UnlimitedWaitCanBeClosedAndAllThreadsAreJoined)
{
  WsFixture server([&](auto & peer, const Json &, std::size_t) {
      peer.send(Json{{"type", "response.created"}, {"response", {{"id", "waiting"}}}});
      server.waitRelease(0);
    });
  FixtureFiles files;
  SessionManager session(files.sessionConfig(server.url()));
  session.addInputs(message("wait")); session.request();
  waitFor([&] {return server.requests().size() == 1;});
  std::this_thread::sleep_for(120ms);
  EXPECT_EQ(session.getRequestStatus().at("state"), "generating");
  const auto begin = std::chrono::steady_clock::now();
  session.shutdown();
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 500ms);
  session.shutdown();
  EXPECT_EQ(session.getRequestStatus().at("state"), "closed");
  EXPECT_EQ(session.getRequestStatus().at("transport").at("state"), "closed");
  EXPECT_THROW(session.addInputs(message("closed")), std::logic_error);
}

TEST(SessionManager, StreamDisplayAndCompactionDoNotConsumeOrRewriteCompleteResponse)
{
  auto full = completed("r1", "A后缀");
  const Json compaction{{"id", "cmp_1"}, {"type", "compaction"}, {"encrypted_content", "压缩密文"}};
  full["output"].insert(full["output"].begin() + 1, compaction);
  full["output"][2]["content"].push_back(Json{{"type", "output_text"}, {"text", "B"}});
  WsFixture server([&](auto & peer, const Json &, std::size_t) {
      peer.send(Json{{"type", "response.created"}, {"response", {{"id", "r1"}}}});
      peer.send(Json{{"type", "response.output_item.done"}, {"output_index", 1}, {"item", compaction}});
      for (const auto & part : std::vector<std::pair<int, std::string>>{{0, "A"}, {1, "B"}, {0, "后缀"}}) {
        peer.send(Json{{"type", "response.output_text.delta"},
            {"output_index", 2}, {"content_index", part.first}, {"delta", part.second}});
      }
      peer.send(Json{{"type", "response.function_call_arguments.delta"}, {"delta", "{"}});
      server.waitRelease(0);
      peer.send(Json{{"type", "response.completed"}, {"response", full}}, true);
    });
  FixtureFiles files;
  SessionManager session(files.sessionConfig(server.url(), {{"stream_output", true}}));
  session.addInputs(message("stream")); session.request();
  waitFor([&] {
    const auto stream = session.getStreamSnapshot();
    return stream.at("content").size() == 2 &&
           stream.at("content").at(0).at("text") == "A后缀";
  });
  EXPECT_EQ(session.getStreamSnapshot().at("response_id"), "r1");
  EXPECT_TRUE(session.takeResponse().is_null());
  EXPECT_EQ(session.getContextStatus().at("latest_response_id"), "");
  server.release(0);
  waitFor([&] {return session.getRequestStatus().at("response_ready").get<bool>();});
  EXPECT_EQ(session.getStreamSnapshot().at("state"), "completed");
  EXPECT_TRUE(session.getRequestStatus().at("response_ready").get<bool>());
  EXPECT_EQ(session.takeResponse(), full);
  EXPECT_EQ(session.getContextStatus().at("compaction_count"), 1);
  EXPECT_EQ(session.getContextStatus().at("usage"), full.at("usage"));
}

TEST(SessionManager, InstancesAndConcurrentInputBatchesStayIndependent)
{
  WsFixture first([](auto & peer, const Json &, std::size_t) {
      peer.send(Json{{"type", "response.completed"}, {"response", completed("first")}});
    });
  WsFixture second([](auto & peer, const Json &, std::size_t) {
      peer.send(Json{{"type", "response.completed"}, {"response", completed("second")}});
    });
  FixtureFiles a, b;
  SessionManager one(a.sessionConfig(first.url())), two(b.sessionConfig(second.url()));
  std::vector<std::thread> writers;
  for (int i = 0; i < 4; ++i) {
    writers.emplace_back([&, i] {
      one.addInputs(Json::array_t{
          message(std::to_string(i) + "a"), message(std::to_string(i) + "b")});
    });
  }
  for (auto & thread : writers) {thread.join();}
  EXPECT_THROW(one.addInputs(Json::array({message("valid"), 1})), std::invalid_argument);
  EXPECT_THROW(one.setTools(Json::object()), std::invalid_argument);
  one.request();
  two.addInputs(message("other")); two.request();
  EXPECT_EQ(take(one).at("id"), "first");
  EXPECT_EQ(take(two).at("id"), "second");
  const auto input = first.requests().at(0).at("input");
  ASSERT_EQ(input.size(), 8u);
  for (std::size_t i = 0; i < input.size(); i += 2) {
    auto text = input[i].at("content").at(0).at("text").get<std::string>();
    text.back() = 'b';
    EXPECT_EQ(input[i + 1].at("content").at(0).at("text"), text);
  }
  EXPECT_FALSE(second.requests().at(0).contains("previous_response_id"));
}

TEST(SessionManager, RejectsInvalidLocalConfigurationBeforeStartingThreads)
{
  FixtureFiles files;
  for (const auto & overrides : std::vector<Json>{
      {{"max_request_hz", 0}}, {{"request_timeout_sec", -1}},
      {{"context", {{"budget_tokens", 0}, {"compact_ratio", 0.8}}}},
      {{"context", {{"budget_tokens", 1000}, {"compact_ratio", 1.0}}}}})
  {
    EXPECT_THROW(SessionManager{files.sessionConfig("http://127.0.0.1:1/v1", overrides)},
      std::invalid_argument);
  }
}

}  // namespace vln_agent::test
