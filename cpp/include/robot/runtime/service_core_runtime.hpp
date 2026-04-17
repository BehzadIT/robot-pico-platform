#pragma once

#include <optional>
#include <string>

#include "robot/runtime/shared_runtime_state.hpp"

namespace robot::runtime {

/**
 * Service-core ingress/runtime facade.
 *
 * This object models the eventual FreeRTOS-facing domain: parse app requests,
 * enqueue them to the control core, and collect completion results. It is
 * intentionally transport-focused and never owns motion policy.
 */
class ServiceCoreRuntime {
 public:
  explicit ServiceCoreRuntime(SharedRuntimeState& shared_state);

  [[nodiscard]] bool QueueDrive(const robot::protocol::ApiDriveRequest& request,
                                const std::string& command_source_id);
  [[nodiscard]] bool QueueStop(std::optional<int> seq,
                               const std::string& command_source_id);
  [[nodiscard]] bool QueueRecover(std::optional<int> seq,
                                  const std::string& command_source_id);
  [[nodiscard]] bool QueueDisconnect(const std::string& command_source_id);

  [[nodiscard]] std::optional<ControlCompletion> PollCompletion();

 private:
  SharedRuntimeState& shared_state_;
};

}  // namespace robot::runtime
