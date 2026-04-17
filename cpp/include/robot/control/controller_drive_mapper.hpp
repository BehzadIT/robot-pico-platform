#pragma once

#include <cstdint>
#include <string>

#include "robot/protocol/joystick_snapshot.hpp"

namespace robot::control {

struct ControllerDriveConfig {
  uint32_t publish_period_ms{50};
  uint32_t snapshot_stale_ms{250};
  int trigger_press_threshold{120};
  int trigger_release_threshold{80};
  int turn_deadzone{240};
};

struct MappedDrive {
  std::string mode;
  std::string reason;
  int throttle_percent{0};
  int turn_percent{0};
  uint32_t snapshot_seq{0};
  int32_t age_ms{0};
};

/**
 * Map transport-oriented joystick samples into robot drive intent.
 *
 * This logic stays in the control domain because dead zones, hysteresis,
 * stale-input meaning, and neutral-vs-driving rules are robot behavior, not
 * transport parsing.
 */
class ControllerDriveMapper {
 public:
  explicit ControllerDriveMapper(ControllerDriveConfig config);

  void Reset();

  [[nodiscard]] MappedDrive MapSnapshot(
      const robot::protocol::JoystickSnapshot* snapshot,
      uint32_t now_ms) noexcept;

 private:
  [[nodiscard]] int MapSignedThrottle(int left_trigger,
                                      int right_trigger) noexcept;
  [[nodiscard]] int MapTurn(int axis_x) const noexcept;

  static int ScaleWithThreshold(int value, int threshold, int maximum) noexcept;
  static int ScaleSignedWithDeadzone(int value,
                                     int deadzone,
                                     int maximum) noexcept;

  ControllerDriveConfig config_;
  bool throttle_active_{false};
};

}  // namespace robot::control
