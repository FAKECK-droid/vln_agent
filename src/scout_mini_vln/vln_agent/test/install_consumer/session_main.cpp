#include <iostream>

#include <vln_agent/session/session_manager.hpp>

/** @brief 只验证独立会话库的安装接口和生命周期，不提交模型请求。 */
int main(int argc, char ** argv)
{
  if (argc != 2) {
    return 2;
  }
  vln_agent::SessionManager session(argv[1]);
  const auto context = session.getContextStatus();
  if (context.at("latest_response_id") != "" || !session.takeResponse().is_null()) {
    return 1;
  }
  session.shutdown();
  std::cout << session.getRequestStatus().at("state") << '\n';
  return session.getRequestStatus().at("state") == "closed" ? 0 : 1;
}
