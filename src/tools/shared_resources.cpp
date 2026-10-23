#include "vln_agent/tools/shared_resources.hpp"

#include <vector>

namespace vln_agent
{

SharedResources::~SharedResources()
{
  shutdown();
}

void SharedResources::shutdown() noexcept
{
  std::vector<std::shared_ptr<SharedResource>> resources;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return;
    }
    closed_ = true;
    for (const auto & item : entries_) {
      resources.push_back(item.second.resource);
    }
  }
  for (const auto & resource : resources) {
    resource->shutdown();
  }
}

}  // namespace vln_agent
