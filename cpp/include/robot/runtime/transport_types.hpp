#pragma once

#include <optional>
#include <string>

#include "robot/protocol/api_drive_request.hpp"

namespace robot::runtime {

enum class TransportCommandKind {
  kDrive,
  kStop,
  kRecover,
  kDisconnect,
};

struct TransportCommand {
  TransportCommandKind kind{TransportCommandKind::kDrive};
  std::string command_source_id;
  std::optional<int> seq;
  robot::protocol::ApiDriveRequest drive_request{};
};

struct ControlCompletion {
  bool ok{false};
  std::string code;
  std::string detail;
  std::optional<int> seq;
};

}  // namespace robot::runtime
