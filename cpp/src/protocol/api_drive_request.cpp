#include "robot/protocol/api_drive_request.hpp"

#include <algorithm>
#include <cmath>

#include "robot/control/constants.hpp"

namespace robot::protocol {

namespace {

int ClampPercent(int value) {
  return std::clamp(value, -100, 100);
}

int ClampAngle(int value) {
  return std::clamp(value,
                    robot::control::SteeringLimits::kAngleMin,
                    robot::control::SteeringLimits::kAngleMax);
}

int NormalizeRpm(int value) {
  if (std::abs(value) < robot::control::MotorLimits::kMinEffectiveRpm) {
    return 0;
  }
  return value;
}

int PercentToSignedRpm(int value) {
  return static_cast<int>(std::lround(
      (value / 100.0) * robot::control::MotorLimits::kRpmMax));
}

int PercentToAngle(int value) {
  return ClampAngle(static_cast<int>(std::lround(
      (value / 100.0) * robot::control::SteeringLimits::kAngleMax)));
}

}  // namespace

ApiDriveRequest ApiDriveRequest::FromIntent(int throttle_percent,
                                            int turn_percent,
                                            std::optional<int> seq,
                                            std::optional<int> sent_at_ms) {
  ApiDriveRequest request;
  request.seq = seq;
  request.sent_at_ms = sent_at_ms;
  request.throttle_percent = ClampPercent(throttle_percent);
  request.turn_percent = ClampPercent(turn_percent);
  const int signed_rpm = PercentToSignedRpm(request.throttle_percent);
  request.target_rpm = NormalizeRpm(std::abs(signed_rpm));
  request.target_direction = signed_rpm >= 0 ? 0 : 1;
  request.target_angle = PercentToAngle(request.turn_percent);
  return request;
}

}  // namespace robot::protocol
