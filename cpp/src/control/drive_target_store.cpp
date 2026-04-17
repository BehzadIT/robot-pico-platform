#include "robot/control/drive_target_store.hpp"

#include <algorithm>

#include "robot/control/constants.hpp"

namespace robot::control {

void DriveTargetStore::Update(const robot::protocol::ApiDriveRequest& drive_request,
                              int now_ms) noexcept {
  target_rpm_ = drive_request.target_rpm;
  target_direction_ = drive_request.target_direction;
  target_angle_ = ClampAngle(drive_request.target_angle);
  if (drive_request.seq.has_value()) {
    last_seq_ = *drive_request.seq;
  }
  last_command_time_ms_ = now_ms;
}

void DriveTargetStore::SetTurnAngle(int angle, int now_ms) noexcept {
  target_angle_ = ClampAngle(angle);
  last_command_time_ms_ = now_ms;
}

void DriveTargetStore::Reset(int now_ms) noexcept {
  target_rpm_ = 0;
  target_angle_ = 0;
  last_command_time_ms_ = now_ms;
}

DriveTarget DriveTargetStore::Snapshot(int now_ms) const noexcept {
  return {
      .target_rpm = target_rpm_,
      .target_angle = target_angle_,
      .target_direction = target_direction_,
      .last_seq = last_seq_,
      .command_age_ms = CommandAgeMs(now_ms),
  };
}

int DriveTargetStore::CommandAgeMs(int now_ms) const noexcept {
  return now_ms - last_command_time_ms_;
}

int DriveTargetStore::ClampAngle(int angle) noexcept {
  return std::clamp(angle,
                    robot::control::SteeringLimits::kAngleMin,
                    robot::control::SteeringLimits::kAngleMax);
}

}  // namespace robot::control
