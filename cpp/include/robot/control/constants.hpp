#pragma once

#include <cstdint>

namespace robot::control {

struct MotorPins {
  static constexpr int kRightPwm = 7;
  static constexpr int kRightDir = 6;
  static constexpr int kLeftPwm = 9;
  static constexpr int kLeftDir = 8;
};

struct MotorLimits {
  static constexpr int kRpmMax = 330;
  static constexpr int kMinEffectiveRpm = 8;
  static constexpr int kPwmMin = 4000;
  static constexpr int kPwmMax = 65535;
  static constexpr int kPwmFreq = 20'000;
};

struct EncoderPins {
  static constexpr int kRightSmId = 0;
  static constexpr int kRightPinA = 10;
  static constexpr int kRightPinB = 11;
  static constexpr int kLeftSmId = 1;
  static constexpr int kLeftPinA = 20;
  static constexpr int kLeftPinB = 21;
};

struct EncoderGeometry {
  static constexpr int kEncoderCpr = 64;
  static constexpr int kGearRatio = 30;
};

struct ControlTiming {
  static constexpr int kControlPeriodUs = 20'000;
  static constexpr int kSummaryPeriodMs = 1'000;
};

struct SafetyTiming {
  static constexpr int kCommandTimeoutMs = 500;
  static constexpr int kTimeoutRestartGuardMs = 150;
  static constexpr int kWatchdogTimeoutMs = 4'000;
  static constexpr int kStopCompletionTimeoutMs = 1'000;
  static constexpr int kRecoverCompletionTimeoutMs = 3'000;
};

struct SteeringLimits {
  static constexpr int kAngleMin = -90;
  static constexpr int kAngleMax = 90;
  static constexpr int kAngleDefault = 0;
};

}  // namespace robot::control
