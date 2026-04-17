#pragma once

namespace robot::control {

/**
 * Snapshot of the accepted robot-level drive target.
 *
 * This value object remains in the control domain. It carries operator intent
 * as accepted by control policy, not final per-track actuator outputs.
 */
struct DriveTarget {
  int target_rpm{0};
  int target_angle{0};
  int target_direction{0};
  int last_seq{-1};
  int command_age_ms{0};

  [[nodiscard]] int signed_target_rpm() const noexcept {
    return target_direction == 0 ? target_rpm : -target_rpm;
  }
};

}  // namespace robot::control
