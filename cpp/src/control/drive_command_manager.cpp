#include "robot/control/drive_command_manager.hpp"

#include <algorithm>
#include <string>

#include "robot/support/logger.hpp"

namespace robot::control {

namespace {

constexpr const char* kDriveTag = "drive";

}  // namespace

DriveCommandManager::DriveCommandManager(int timeout_restart_guard_ms)
    : timeout_guard_until_ms_(0),
      timeout_restart_guard_ms_(timeout_restart_guard_ms) {}

CommandResult DriveCommandManager::AcceptDriveRequest(
    const robot::protocol::ApiDriveRequest& drive_request,
    int now_ms,
    std::optional<std::string> command_source_id) {
  if (command_source_id.has_value() && active_command_source_id_.has_value() &&
      *active_command_source_id_ != *command_source_id) {
    return {.ok = false,
            .code = "ic",
            .detail = "command source does not own drivetrain"};
  }

  if (drive_request.seq.has_value() &&
      *drive_request.seq <= std::max(drive_target_store_.last_seq(), timeout_fence_seq_)) {
    return {.ok = false, .code = "st", .detail = "stale sequence"};
  }

  if (timeout_guard_until_ms_ > now_ms) {
    return {.ok = false, .code = "si", .detail = "timeout stop settling"};
  }

  drive_target_store_.Update(drive_request, now_ms);
  active_command_source_id_ = command_source_id;
  return {.ok = true};
}

bool DriveCommandManager::RevokeDriveRequest(
    int now_ms,
    std::optional<int> seq,
    std::optional<std::string> command_source_id) {
  if (command_source_id.has_value() && active_command_source_id_.has_value() &&
      *active_command_source_id_ != *command_source_id) {
    return false;
  }
  if (seq.has_value() && drive_target_store_.last_seq() != *seq) {
    return false;
  }
  drive_target_store_.Reset(now_ms);
  active_command_source_id_.reset();
  robot::support::LogW(kDriveTag, "drive request revoked");
  return true;
}

void DriveCommandManager::SetTurnAngle(int angle, int now_ms) noexcept {
  drive_target_store_.SetTurnAngle(angle, now_ms);
}

bool DriveCommandManager::Stop(
    int now_ms,
    std::optional<std::string> command_source_id) noexcept {
  if (command_source_id.has_value() && active_command_source_id_.has_value() &&
      *active_command_source_id_ != *command_source_id) {
    robot::support::LogW(kDriveTag, "ignoring stop from non-active command source");
    return false;
  }
  drive_target_store_.Reset(now_ms);
  active_command_source_id_.reset();
  robot::support::LogI(kDriveTag, "drive stop requested");
  return true;
}

DriveTarget DriveCommandManager::GetDriveTarget(int now_ms) const noexcept {
  return drive_target_store_.Snapshot(now_ms);
}

bool DriveCommandManager::IsTimeout(int now_ms, int timeout_ms) const noexcept {
  if (drive_target_store_.target_rpm() == 0) {
    return false;
  }
  return drive_target_store_.CommandAgeMs(now_ms) > timeout_ms;
}

int DriveCommandManager::CommandAgeMs(int now_ms) const noexcept {
  return drive_target_store_.CommandAgeMs(now_ms);
}

bool DriveCommandManager::IsFreshSequence(
    std::optional<int> seq,
    std::optional<std::string> command_source_id) const noexcept {
  if (!seq.has_value()) {
    return true;
  }
  if (command_source_id.has_value() && active_command_source_id_.has_value() &&
      *active_command_source_id_ != *command_source_id) {
    return false;
  }
  return *seq > std::max(drive_target_store_.last_seq(), timeout_fence_seq_);
}

bool DriveCommandManager::ClearCommandSourceIfActive(
    const std::string& command_source_id,
    int now_ms) noexcept {
  if (!active_command_source_id_.has_value() ||
      *active_command_source_id_ != command_source_id) {
    return false;
  }
  drive_target_store_.Reset(now_ms);
  active_command_source_id_.reset();
  robot::support::LogW(kDriveTag, std::string("active command source lost: ") + command_source_id);
  return true;
}

std::optional<TimeoutStopResult> DriveCommandManager::TimeoutStop(
    int now_ms,
    int timeout_ms) noexcept {
  if (drive_target_store_.target_rpm() == 0) {
    return std::nullopt;
  }

  const int age_ms = drive_target_store_.CommandAgeMs(now_ms);
  if (age_ms <= timeout_ms) {
    return std::nullopt;
  }

  const TimeoutStopResult snapshot{
      .age_ms = age_ms,
      .last_seq = drive_target_store_.last_seq(),
      .command_source_id = active_command_source_id_.value_or(""),
  };

  const int last_seq = drive_target_store_.last_seq();
  drive_target_store_.Reset(now_ms);
  if (last_seq >= 0) {
    timeout_fence_seq_ = std::max(timeout_fence_seq_, last_seq);
  }
  active_command_source_id_.reset();
  timeout_guard_until_ms_ = now_ms + timeout_restart_guard_ms_;
  robot::support::LogW(kDriveTag, "command timeout reached; stopping drive command manager");
  return snapshot;
}

}  // namespace robot::control
