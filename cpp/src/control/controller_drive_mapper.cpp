#include "robot/control/controller_drive_mapper.hpp"

#include <algorithm>
#include <cstdlib>

namespace robot::control {

ControllerDriveMapper::ControllerDriveMapper(ControllerDriveConfig config)
    : config_(config) {}

void ControllerDriveMapper::Reset() { throttle_active_ = false; }

MappedDrive ControllerDriveMapper::MapSnapshot(
    const robot::protocol::JoystickSnapshot* snapshot,
    uint32_t now_ms) noexcept {
  if (snapshot == nullptr) {
    Reset();
    return {.mode = "inactive", .reason = "no_snapshot"};
  }

  const int32_t age_ms = static_cast<int32_t>(now_ms - snapshot->received_at_ms);
  if (age_ms > static_cast<int32_t>(config_.snapshot_stale_ms)) {
    Reset();
    return {.mode = "inactive", .reason = "snapshot_stale", .age_ms = age_ms};
  }

  if (!snapshot->connected) {
    Reset();
    return {.mode = "inactive",
            .reason = "controller_disconnected",
            .age_ms = age_ms};
  }

  const int throttle_percent =
      MapSignedThrottle(snapshot->left_trigger, snapshot->right_trigger);
  const int turn_percent = MapTurn(snapshot->left_stick_x);

  if (throttle_percent == 0) {
    return {.mode = "armed_neutral",
            .reason = "neutral_throttle",
            .throttle_percent = 0,
            .turn_percent = turn_percent,
            .snapshot_seq = snapshot->seq,
            .age_ms = age_ms};
  }

  return {.mode = "driving",
          .throttle_percent = throttle_percent,
          .turn_percent = turn_percent,
          .snapshot_seq = snapshot->seq,
          .age_ms = age_ms};
}

int ControllerDriveMapper::MapSignedThrottle(int left_trigger,
                                             int right_trigger) noexcept {
  const int signed_raw = right_trigger - left_trigger;
  const int magnitude = std::abs(signed_raw);
  const int sign = signed_raw >= 0 ? 1 : -1;

  int threshold = config_.trigger_press_threshold;
  if (throttle_active_) {
    if (magnitude < config_.trigger_release_threshold) {
      throttle_active_ = false;
      return 0;
    }
    threshold = config_.trigger_release_threshold;
  } else {
    if (magnitude < config_.trigger_press_threshold) {
      return 0;
    }
    throttle_active_ = true;
  }

  const int scaled = ScaleWithThreshold(magnitude, threshold, 1000);
  return std::clamp(sign * scaled, -100, 100);
}

int ControllerDriveMapper::MapTurn(int axis_x) const noexcept {
  return ScaleSignedWithDeadzone(axis_x, config_.turn_deadzone, 1000);
}

int ControllerDriveMapper::ScaleWithThreshold(int value,
                                              int threshold,
                                              int maximum) noexcept {
  if (value <= threshold) {
    return 0;
  }

  const int usable_range = maximum - threshold;
  if (usable_range <= 0) {
    return 100;
  }

  const int scaled = static_cast<int>(
      ((value - threshold) * 100.0) / static_cast<double>(usable_range) + 0.5);
  return std::clamp(scaled, 0, 100);
}

int ControllerDriveMapper::ScaleSignedWithDeadzone(int value,
                                                   int deadzone,
                                                   int maximum) noexcept {
  const int sign = value >= 0 ? 1 : -1;
  const int magnitude = std::abs(value);
  return sign * ScaleWithThreshold(magnitude, deadzone, maximum);
}

}  // namespace robot::control
