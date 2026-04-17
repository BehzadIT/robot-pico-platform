#include "robot/runtime/service_core_runtime.hpp"

namespace robot::runtime {

ServiceCoreRuntime::ServiceCoreRuntime(SharedRuntimeState& shared_state)
    : shared_state_(shared_state) {}

bool ServiceCoreRuntime::QueueDrive(const robot::protocol::ApiDriveRequest& request,
                                    const std::string& command_source_id) {
  return shared_state_.inbound_commands.Push(TransportCommand{
      .kind = TransportCommandKind::kDrive,
      .command_source_id = command_source_id,
      .seq = request.seq,
      .drive_request = request,
  });
}

bool ServiceCoreRuntime::QueueStop(std::optional<int> seq,
                                   const std::string& command_source_id) {
  return shared_state_.inbound_commands.Push(TransportCommand{
      .kind = TransportCommandKind::kStop,
      .command_source_id = command_source_id,
      .seq = seq,
  });
}

bool ServiceCoreRuntime::QueueRecover(std::optional<int> seq,
                                      const std::string& command_source_id) {
  return shared_state_.inbound_commands.Push(TransportCommand{
      .kind = TransportCommandKind::kRecover,
      .command_source_id = command_source_id,
      .seq = seq,
  });
}

bool ServiceCoreRuntime::QueueDisconnect(const std::string& command_source_id) {
  return shared_state_.inbound_commands.Push(TransportCommand{
      .kind = TransportCommandKind::kDisconnect,
      .command_source_id = command_source_id,
  });
}

std::optional<ControlCompletion> ServiceCoreRuntime::PollCompletion() {
  return shared_state_.outbound_completions.Pop();
}

}  // namespace robot::runtime
