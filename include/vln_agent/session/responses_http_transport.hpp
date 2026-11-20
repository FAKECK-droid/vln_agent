#pragma once

#include <memory>

#include "vln_agent/session/responses_transport.hpp"

namespace vln_agent
{
/** @brief HTTP POST /responses，通过 SSE 交付原生 Responses 事件。 */
class ResponsesHttpTransport : public ResponsesTransport
{
public:
  ResponsesHttpTransport();
  ~ResponsesHttpTransport() override;
  void connect(const std::string & base_url, const std::string & api_key,
    EventHandler on_event, StateHandler on_state) override;
  void send(Json request) override;
  Json getStatus() const override;
  void interrupt() override;
  void close() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace vln_agent
