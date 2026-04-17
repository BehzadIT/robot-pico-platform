#pragma once

#include "robot/control/drive_target.hpp"
#include "robot/protocol/api_drive_request.hpp"

namespace robot::control {

/**
 * Mutable store for the latest accepted drivetrain target state.
 *
 * This type owns robot-level target values and freshness timestamp only. It
 * does not decide who may update the target; that policy stays in
 * DriveCommandManager.
 */
class DriveTargetStore {
 public:
  /** Store one accepted target and refresh its command timestamp. */
  void Update(const robot::protocol::ApiDriveRequest& drive_request,
              int now_ms) noexcept;
  /** Update steering only while preserving the last accepted drive source. */
  void SetTurnAngle(int angle, int now_ms) noexcept;
  /** Reset motion intent to safe neutral without changing the sequence fence. */
  void Reset(int now_ms) noexcept;

  /** Snapshot the robot-level target state consumed by the control loop. */
  [[nodiscard]] DriveTarget Snapshot(int now_ms) const noexcept;
  /** Return age in milliseconds using caller-supplied monotonic time. */
  [[nodiscard]] int CommandAgeMs(int now_ms) const noexcept;
  [[nodiscard]] int target_rpm() const noexcept { return target_rpm_; }
  [[nodiscard]] int last_seq() const noexcept { return last_seq_; }

 private:
  static int ClampAngle(int angle) noexcept;

  int target_rpm_{0};
  int target_angle_{0};
  int target_direction_{0};
  int last_command_time_ms_{0};
  int last_seq_{-1};
};

}  // namespace robot::control
