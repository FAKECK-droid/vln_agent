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
  SessionManager session(files.sessionConfig(server.url(),
    {{"parallel_tool_calls", false}, {"prompt_cache_key", "fixture"}, {"forward_tool_async", true}}));
  ready(session);
  EXPECT_EQ(session.getContextStatus().at("mode"), "server");
  EXPECT_EQ(session.getContextStatus().at("history_item_count"), 0);
  EXPECT_TRUE(session.getTools().empty());
  EXPECT_TRUE(session.getPendingInputs().empty());
  EXPECT_TRUE(session.getCurrentRequest().is_null());
  EXPECT_EQ(session.getRequestStatus().at("request_elapsed_ms"), 0);
  EXPECT_EQ(session.getRequestStatus().at("rate_limit_remaining_ms"), 0);
  EXPECT_FALSE(session.getRequestStatus().at("retry_required").get<bool>());
  session.setTools(Json::array({{{"type", "function"}, {"name", "example"}, {"async", true}}}));
  session.request();
  session.addInputs(message("系统规则", "system"));
  session.addInputs(Json::array_t{message("用户"), message("小车状态")});
  auto returned = utils::businessPayload("robot");
  returned["ok"] = true;
  const auto tool_output = utils::functionCallOutput("c1", returned, {});
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
  EXPECT_EQ(request.at("parallel_tool_calls"), false);
  EXPECT_EQ(request.at("prompt_cache_key"), "fixture");
  EXPECT_EQ(request.at("tools")[0].at("async"), true);
  EXPECT_FALSE(request.contains("reasoning"));
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
  waitFor([&] {return session.getRequestStatus().at("current_response_id") == "r1";});
  auto current = session.getCurrentRequest();
  auto frame = server.requests().at(0);
  EXPECT_EQ(frame.at("type"), "response.create");
  frame.erase("type");
  EXPECT_EQ(current, frame);
  EXPECT_EQ(current.at("input"), Json::array({message("first")}));
  session.setTools(Json::array({{{"name", "new"}}}));
  session.addInputs(Json::array_t{message("second"), message("third")});
  session.request();
  session.request();
  EXPECT_EQ(session.getRequestStatus().at("pending_input_count"), 2);
  auto pending = session.getPendingInputs();
  auto tools = session.getTools();
  EXPECT_EQ(pending, Json::array({message("second"), message("third")}));
  EXPECT_EQ(tools, Json::array({{{"name", "new"}}}));
  EXPECT_EQ(session.getCurrentRequest().at("tools").at(0).at("name"), "old");
  const auto elapsed = session.getRequestStatus().at("request_elapsed_ms").get<int>();
  current["input"].clear();
  pending.clear();
  tools.clear();
  EXPECT_EQ(session.getCurrentRequest().at("input").size(), 1u);
  EXPECT_EQ(session.getPendingInputs().size(), 2u);
  EXPECT_EQ(session.getTools().size(), 1u);
  EXPECT_FALSE(session.getRequestStatus().at("can_retry").get<bool>());
  std::this_thread::sleep_for(15ms);
  EXPECT_GE(session.getRequestStatus().at("request_elapsed_ms").get<int>(), elapsed + 10);
  server.release(0);
  waitFor([&] {return session.getRequestStatus().at("response_ready").get<bool>();});
  const auto finished_elapsed = session.getRequestStatus().at("request_elapsed_ms");
  EXPECT_TRUE(session.getCurrentRequest().is_null());
  EXPECT_FALSE(session.getRequestStatus().at("retry_required").get<bool>());
  EXPECT_EQ(session.getRequestStatus().at("current_response_id"), "r1");
  EXPECT_EQ(session.getPendingInputs().size(), 2u);
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(session.getRequestStatus().at("request_elapsed_ms"), finished_elapsed);
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
  const auto waiting = session.getRequestStatus();
  EXPECT_EQ(waiting.at("state"), "waiting_interval");
  EXPECT_GT(waiting.at("rate_limit_remaining_ms").get<int>(), 0);
  EXPECT_LE(waiting.at("rate_limit_remaining_ms").get<int>(), 250);
  // 只读取服务端观测，不调用任何能唤醒会话调度器的方法。
  waitFor([&] {return server.requests().size() == 2;});
  const auto starts = server.starts();
  EXPECT_GE(std::chrono::duration<double>(starts[1] - starts[0]).count(), 0.20);
  EXPECT_EQ(take(session).at("id"), "r2");
}

TEST(SessionManager, ModelErrorsKeepInputsAndRequireExplicitRetry)
{
  for (const std::string event : {"error", "response.failed", "response.incomplete", "http_error"}) {
    SCOPED_TRACE(event);
    WsServerOptions options;
    options.http_sse = event == "http_error";
    const std::string type = options.http_sse ? "error" : event;
    const Json error{{"code", "model_error"}, {"message", "测试失败"}};
    WsFixture server([&](auto & peer, const Json &, std::size_t index) {
        if (index == 1) {
          server.waitRelease(1);
          if (options.http_sse) {
            auto streamed_error = error;
            streamed_error["type"] = "error";
            peer.send(streamed_error);
          } else if (type == "error") {
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
      }, options);
    FixtureFiles files;
    const char * transport = options.http_sse ?
      "vln_agent/ResponsesHttpTransport" : "vln_agent/ResponsesWsTransport";
    SessionManager session(files.sessionConfig(server.url(), {{"transport_plugin", transport}}));
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
    EXPECT_TRUE(status.at("retry_required").get<bool>());
    EXPECT_TRUE(status.at("can_retry").get<bool>());
    EXPECT_EQ(session.getCurrentRequest().at("input"), Json::array({message("failed")}));
    EXPECT_EQ(session.getPendingInputs(), Json::array({message("later")}));
    EXPECT_EQ(status.at("last_error").at("event").at("type"), type);
    EXPECT_EQ(status.at("last_error").at("code"), "model_error");
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
  const auto status = session.getRequestStatus();
  EXPECT_EQ(status.at("last_error").at("code"), "previous_response_not_found");
  EXPECT_TRUE(status.at("retry_required").get<bool>());
  EXPECT_FALSE(status.at("can_retry").get<bool>());
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
  EXPECT_TRUE(session.getRequestStatus().at("retry_required").get<bool>());
  EXPECT_TRUE(session.getRequestStatus().at("can_retry").get<bool>());
  EXPECT_EQ(session.getRequestStatus().at("current_response_id"), "r2");
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
  for (const bool http_sse : {false, true}) {
    SCOPED_TRACE(http_sse ? "HTTP SSE" : "WebSocket");
    WsServerOptions options;
    options.http_sse = http_sse;
    WsFixture server([&](auto & peer, const Json &, std::size_t index) {
        if (index == 1) {
          peer.send(Json{{"type", "response.created"}, {"response", {{"id", "late"}}}});
          server.waitRelease(1);
          peer.send(Json{{"type", "response.completed"}, {"response", completed("late")}});
        } else {
          peer.send(Json{{"type", "response.completed"},
              {"response", completed("r" + std::to_string(index + 1))}});
        }
      }, options);
    FixtureFiles files;
    const char * transport = http_sse ?
      "vln_agent/ResponsesHttpTransport" : "vln_agent/ResponsesWsTransport";
    SessionManager session(files.sessionConfig(server.url(),
      {{"transport_plugin", transport}, {"request_timeout_sec", 0.1}}));
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
  EXPECT_FALSE(session.getRequestStatus().at("can_retry").get<bool>());
  EXPECT_FALSE(session.getRequestStatus().at("retry_required").get<bool>());
  EXPECT_TRUE(session.getCurrentRequest().is_null());
  EXPECT_TRUE(session.getPendingInputs().empty());
  const auto elapsed = session.getRequestStatus().at("request_elapsed_ms");
  std::this_thread::sleep_for(15ms);
  EXPECT_EQ(session.getRequestStatus().at("request_elapsed_ms"), elapsed);
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
  EXPECT_TRUE(session.getStreamSnapshot().at("content").empty());
  EXPECT_EQ(session.getStreamSnapshot().at("response_id"), "");
  EXPECT_EQ(session.getStreamSnapshot().at("state"), "idle");
  EXPECT_EQ(session.getContextStatus().at("latest_response_id"), "r1");
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


TEST(SessionManager, HttpStreamsAndReplaysNativeMultimodalToolResults)
{
  WsServerOptions options;
  options.http_sse = true;
  auto first = completed("http1", "看到门");
  first["output"].push_back({{"type", "function_call"}, {"call_id", "c-http"},
      {"name", "capture_pair"}, {"arguments", "{}"}});
  WsFixture server([&](auto & peer, const Json &, std::size_t index) {
      if (index == 0) {
        peer.send({{"type", "response.created"}, {"response", {{"id", "http1"}}}});
        peer.send({{"type", "response.output_text.delta"}, {"output_index", 1},
            {"content_index", 0}, {"delta", "看到门"}}, true);
        server.waitRelease(0);
      }
      peer.send({{"type", "response.completed"}, {"response",
          index == 0 ? first : completed("http2")}}, true);
    }, options);
  FixtureFiles files;
  SessionManager session(files.sessionConfig(server.url(),
    {{"transport_plugin", "vln_agent/ResponsesHttpTransport"}, {"model", "vision-model"},
      {"reasoning_effort", "high"}, {"stream_output", true}, {"context", {{"mode", "local"}}}}));
  session.setTools(Json::array({{{"type", "function"}, {"name", "capture_pair"}, {"async", true}}}));
  session.addInputs(message("导航开始", "system"));
  session.request();
  waitFor([&] {return !session.getStreamSnapshot().at("content").empty();});
  EXPECT_EQ(session.getStreamSnapshot().at("content")[0].at("text"), "看到门");
  EXPECT_TRUE(session.takeResponse().is_null());
  server.release(0);
  EXPECT_EQ(take(session), first);

  auto body = utils::businessPayload("robot");
  body["images"] = Json::array({Json::object()});
  const auto image = utils::functionCallOutput("c-http", body,
      {{{"type", "input_image"}, {"image_url", "data:image/jpeg;base64,AA=="}}});
  session.addInputs(image);
  session.request();
  EXPECT_EQ(take(session).at("id"), "http2");
  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 2u);
  EXPECT_EQ(server.paths(), (std::vector<std::string>{"/v1/responses", "/v1/responses"}));
  EXPECT_EQ(server.authorizations().front(), "Bearer loopback-fixture");
  for (const auto & req : requests) {
    EXPECT_EQ(req.at("stream"), true);
    EXPECT_EQ(req.at("model"), "vision-model");
    EXPECT_EQ(req.at("reasoning").at("effort"), "high");
    for (const auto * field : {"type", "previous_response_id", "context_management", "store",
        "prompt_cache_key", "include", "parallel_tool_calls"})
    {
      EXPECT_FALSE(req.contains(field));
    }
    EXPECT_FALSE(req.at("tools")[0].contains("async"));
  }
  EXPECT_EQ(requests[1].at("input").back(), image);
  EXPECT_EQ(requests[1].at("input").at(requests[1].at("input").size() - 2), first.at("output").back());
}

TEST(SessionManager, LocalHistoryReplaysNativeOutputsToolsAndImagesWithoutStoredIds)
{
  auto first = completed("r1");
  first["output"].push_back(Json{{"type", "function_call"}, {"name", "echo"},
    {"call_id", "c1"}, {"arguments", "{}"}, {"status", "completed"}});
  WsFixture server([&](auto & peer, const Json &, std::size_t index) {
      if (index == 1) {server.waitRelease(1);}
      peer.send(Json{{"type", "response.completed"}, {"response",
          index == 0 ? first : completed("r" + std::to_string(index + 1))}});
    });
  FixtureFiles files;
  SessionManager session(files.sessionConfig(server.url(),
    {{"context", {{"mode", "local"}}}, {"store", false},
      {"include", Json::array({"reasoning.encrypted_content"})}}));
  const Json image{{"type", "input_image"}, {"image_url", "data:image/jpeg;base64,AA=="}};
  auto business = utils::businessPayload("user");
  business["instruction"] = "检查图像";
  business["images"] = Json::array({{{"metadata", {{"label", "scene"}}}}});
  auto user = message(business.dump());
  user["content"].push_back(image);
  const Json initial = Json::array({message("系统规则", "system"), user});
  session.addInputs(initial); session.request();
  EXPECT_EQ(take(session), first);
  Json history = initial;
  utils::appendInputs(history, first.at("output"));
  EXPECT_EQ(session.getContextStatus().at("mode"), "local");
  EXPECT_EQ(session.getContextStatus().at("history_item_count"), history.size());
  auto returned = utils::businessPayload("robot");
  returned["ok"] = true;
  returned["images"] = business.at("images");
  const auto result = utils::functionCallOutput("c1", returned, {image});
  session.addInputs(result); session.request();
  waitFor([&] {return server.requests().size() == 2;});
  session.addInputs(message("下一轮"));
  session.request();
  EXPECT_EQ(session.getPendingInputs(), Json::array({message("下一轮")}));
  EXPECT_EQ(session.getContextStatus().at("history_item_count"), history.size());
  server.release(1);
  EXPECT_EQ(take(session), completed("r2"));
  utils::appendInputs(history, result);
  utils::appendInputs(history, completed("r2").at("output"));
  EXPECT_EQ(take(session), completed("r3"));
  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 3u);
  EXPECT_EQ(requests[0].at("input"), initial);
  auto second_input = initial;
  utils::appendInputs(second_input, first.at("output"));
  utils::appendInputs(second_input, result);
  EXPECT_EQ(requests[1].at("input"), second_input);
  utils::appendInputs(history, message("下一轮"));
  EXPECT_EQ(requests[2].at("input"), history);
  for (const auto & request : requests) {
    EXPECT_EQ(request.at("store"), false);
    EXPECT_EQ(request.at("include"), Json::array({"reasoning.encrypted_content"}));
    EXPECT_FALSE(request.contains("previous_response_id"));
    EXPECT_FALSE(request.contains("context_management"));
  }
  session.shutdown();
  EXPECT_EQ(session.getContextStatus().at("history_item_count"), 0);
}

TEST(SessionManager, LocalHistoryRetriesOnlyFreshInputsAndNeverCommitsPartialOutputs)
{
  for (const std::string type : {"error", "response.failed", "response.incomplete"}) {
    WsFixture server([&](auto & peer, const Json &, std::size_t index) {
        if (index == 1) {
          server.waitRelease(1);
          const Json error{{"code", "model_error"}, {"message", "测试失败"}};
          if (type == "error") {
            peer.send(Json{{"type", type}, {"error", error}});
          } else {
            peer.send(Json{{"type", type}, {"response", {{"id", "failed"}, {"error", error},
              {"output", Json::array({message("不应保存的部分输出", "assistant")})}}}});
          }
        } else {
          peer.send(Json{{"type", "response.completed"},
            {"response", completed("r" + std::to_string(index + 1))}});
        }
      });
    FixtureFiles files;
    SessionManager session(files.sessionConfig(server.url(),
      {{"context", {{"mode", "local"}}}}));
    session.addInputs(message("init", "system")); session.request(); take(session);
    Json history = Json::array({message("init", "system")});
    utils::appendInputs(history, completed("r1").at("output"));
    session.addInputs(message("failed")); session.request();
    waitFor([&] {return server.requests().size() == 2;});
    session.addInputs(message("later"));
    server.release(1);
    waitFor([&] {return session.getRequestStatus().at("state") == "error";});
    EXPECT_EQ(session.getContextStatus().at("history_item_count"), history.size());
    EXPECT_TRUE(session.takeResponse().is_null());
    session.request();
    EXPECT_EQ(take(session).at("id"), "r3");
    utils::appendInputs(history, Json::array({message("failed"), message("later")}));
    const auto retry = server.requests().at(2);
    EXPECT_EQ(retry.at("input"), history);
    EXPECT_FALSE(retry.contains("previous_response_id"));
  }
}

TEST(SessionManager, CompactionCommitsOnlySuccessfulCloudSummaryAndPreservesToolContext)
{
  const auto original = message("系统规则", "system");
  const auto old = message(std::string(1450, 'x'));
  auto recent = completed("recent", "发现门口");
  recent["output"].push_back({{"type", "function_call"}, {"call_id", "c1"},
      {"name", "camera"}, {"arguments", "{}"}});
  WsFixture server([&](auto & peer, const Json &, std::size_t index) {
      if (index == 2) {
        peer.send({{"type", "response.completed"}, {"response", completed("bad", "not-json")}});
      } else if (index == 3) {
        peer.send({{"type", "response.completed"}, {"response",
            completed("summary", R"({"summary":"目标是经过门口"})")}});
      } else {
        peer.send({{"type", "response.completed"}, {"response",
            index == 1 ? recent : completed("r" + std::to_string(index))}});
      }
    });
  FixtureFiles files;
  SessionManager session(files.sessionConfig(server.url(),
    {{"max_output_tokens", 128}, {"reasoning_effort", "high"},
      {"context", {{"mode", "local"}, {"budget_tokens", 2400}, {"compact_ratio", 0.6},
        {"keep_recent_items", 2}, {"summary_max_output_tokens", 128}}}}));
  session.addInputs(Json::array({original, old}));
  session.request();
  EXPECT_EQ(take(session).at("id"), "r0");
  session.addInputs(message("在门口停下"));
  session.request();
  EXPECT_EQ(take(session).at("id"), "recent");

  auto metadata = utils::businessPayload("robot");
  metadata["images"] = Json::array({Json::object()});
  const auto output = utils::functionCallOutput("c1", metadata,
      {{{"type", "input_image"}, {"image_url", "data:image/jpeg;base64,AA=="}}});
  const auto prior = session.getContextStatus().at("history_item_count");
  session.addInputs(output);
  session.addInputs(message(std::string(440, 'y')));
  session.request();
  waitFor([&] {return session.getRequestStatus().at("state") == "error";});
  EXPECT_EQ(session.getRequestStatus().at("last_error").at("code"), "CONTEXT_SUMMARY_FAILED");
  EXPECT_EQ(session.getContextStatus().at("history_item_count"), prior);
  EXPECT_EQ(session.getContextStatus().at("compaction_count"), 0);
  EXPECT_TRUE(session.takeResponse().is_null());

  session.addInputs(message("仍然继续"));
  session.request();
  EXPECT_EQ(take(session).at("id"), "r4");
  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 5u);
  for (int i : {2, 3}) {
    EXPECT_EQ(requests[i].at("reasoning").at("effort"), "high");
    EXPECT_FALSE(requests[i].contains("store"));
    EXPECT_FALSE(requests[i].contains("tools"));
    EXPECT_FALSE(requests[i].contains("previous_response_id"));
    EXPECT_TRUE(requests[i].at("input").dump().find("data:image") == std::string::npos);
  }
  const auto & input = requests[4].at("input");
  EXPECT_EQ(input.front(), original);
  EXPECT_TRUE(input.dump().find("目标是经过门口") != std::string::npos);
  EXPECT_TRUE(input.dump().find(std::string(1450, 'x')) == std::string::npos);
  EXPECT_EQ(input.at(input.size() - 3), output);
  EXPECT_EQ(input.back(), message("仍然继续"));
  EXPECT_EQ(session.getContextStatus().at("compaction_count"), 1);
}

TEST(SessionManager, LocalHistorySurvivesReconnectWithoutServerStoredContext)
{
  WsFixture server([&](auto & peer, const Json &, std::size_t index) {
      if (index == 1) {
        server.waitRelease(1);
        peer.drop();
      } else {
        peer.send(Json{{"type", "response.completed"},
          {"response", completed("r" + std::to_string(index + 1))}});
      }
    });
  FixtureFiles files;
  SessionManager session(files.sessionConfig(server.url(),
    {{"context", {{"mode", "local"}}}}));
  session.addInputs(message("init", "system")); session.request(); take(session);
  session.addInputs(message("interrupted")); session.request();
  waitFor([&] {return server.requests().size() == 2;});
  session.addInputs(message("new"));
  server.release(1);
  waitFor([&] {
    const auto status = session.getRequestStatus();
    return status.at("retry_required").get<bool>() &&
           status.at("transport").at("connected").get<bool>() &&
           status.at("transport").at("connection_id").get<int>() >= 2;
  });
  EXPECT_EQ(server.requests().size(), 2u);
  session.request();
  EXPECT_EQ(take(session).at("id"), "r3");
  Json expected = Json::array({message("init", "system")});
  utils::appendInputs(expected, completed("r1").at("output"));
  utils::appendInputs(expected, Json::array({message("interrupted"), message("new")}));
  EXPECT_EQ(server.requests()[2].at("input"), expected);
  EXPECT_FALSE(server.requests()[2].contains("previous_response_id"));
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
