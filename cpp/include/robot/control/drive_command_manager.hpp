#pragma once

#include <optional>
#include <string>

#include "robot/control/drive_target_store.hpp"

namespace robot::control {

struct CommandResult {
  bool ok{false};
  std::string code;
  std::string detail;
};

struct TimeoutStopResult {
  int age_ms{0};
  int last_seq{-1};
  std::string command_source_id;
};

/**
 * Admit drive commands and track the active command source.
 *
 * This class is control-policy code. It owns source fencing, stale-sequence
 * rejection, timeout restart guards, and target handoff into DriveTargetStore.
 */
class DriveCommandManager {
 public:
  explicit DriveCommandManager(int timeout_restart_guard_ms = 150);

  /**
   * Admit one drive request through control policy.
   *
   * This preserves the MicroPython rules that mattered operationally:
   * - one active command source owns the drivetrain at a time
   * - stale or fenced sequences are rejected
   * - timeout-triggered stops create a short restart guard window so near-edge
   *   in-flight frames cannot immediately re-arm motion
   */
  [[nodiscard]] CommandResult AcceptDriveRequest(
      const robot::protocol::ApiDriveRequest& drive_request,
      int now_ms,
      std::optional<std::string> command_source_id = std::nullopt);

  /** Revoke an already accepted request if its source and sequence still own it. */
  [[nodiscard]] bool RevokeDriveRequest(
      int now_ms,
      std::optional<int> seq = std::nullopt,
      std::optional<std::string> command_source_id = std::nullopt);

  /** Compatibility hook for steering-only updates at the target-store level. */
  void SetTurnAngle(int angle, int now_ms) noexcept;

  /** Clear accepted motion intent, respecting active-source ownership if set. */
  [[nodiscard]] bool Stop(
      int now_ms,
      std::optional<std::string> command_source_id = std::nullopt) noexcept;

  [[nodiscard]] DriveTarget GetDriveTarget(int now_ms) const noexcept;
  [[nodiscard]] bool IsTimeout(int now_ms, int timeout_ms) const noexcept;
  [[nodiscard]] int CommandAgeMs(int now_ms) const noexcept;

  /** Return whether a sequence may be accepted without violating fences. */
  [[nodiscard]] bool IsFreshSequence(
      std::optional<int> seq,
      std::optional<std::string> command_source_id = std::nullopt) const noexcept;

  /** Drop the active source on disconnect-style events if it still owns motion. */
  [[nodiscard]] bool ClearCommandSourceIfActive(const std::string& command_source_id,
                                                int now_ms) noexcept;

  /**
   * Apply timeout-stop policy from the control owner.
   *
   * This path is intentionally authoritative: it stops motion, records the
   * last accepted sequence into the timeout fence, and opens the short restart
   * guard that prevents stale in-flight traffic from immediately resuming
   * motion.
   */
  [[nodiscard]] std::optional<TimeoutStopResult> TimeoutStop(
      int now_ms,
      int timeout_ms) noexcept;

  [[nodiscard]] std::optional<std::string> active_command_source_id() const {
    return active_command_source_id_;
  }

 private:
  DriveTargetStore drive_target_store_;
  std::optional<std::string> active_command_source_id_;
  int timeout_fence_seq_{-1};
  int timeout_guard_until_ms_{0};
  int timeout_restart_guard_ms_{150};
};

}  // namespace robot::control
