#include "robot/control/pid.hpp"

namespace robot::control {

Pid::Pid(double kp,
         double ki,
         double kd,
         double setpoint,
         PidLimits output_limits,
         PidLimits integral_limits)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      setpoint_(setpoint),
      output_limits_(output_limits),
      integral_limits_(integral_limits) {}

double Pid::Update(double measurement, double dt_seconds) {
  if (!auto_mode_) {
    return Clamp(proportional_ + integral_ + derivative_, output_limits_);
  }

  const double error = setpoint_ - measurement;
  proportional_ = kp_ * error;
  integral_ = Clamp(integral_ + (ki_ * error * dt_seconds), integral_limits_);

  if (has_last_input_ && dt_seconds > 0.0) {
    derivative_ = -kd_ * ((measurement - last_input_) / dt_seconds);
  } else {
    derivative_ = 0.0;
  }

  last_input_ = measurement;
  has_last_input_ = true;
  return Clamp(proportional_ + integral_ + derivative_, output_limits_);
}

void Pid::Reset() noexcept {
  proportional_ = 0.0;
  integral_ = 0.0;
  derivative_ = 0.0;
  has_last_input_ = false;
  last_input_ = 0.0;
}

void Pid::SetAutoMode(bool enabled, double last_output) noexcept {
  if (enabled && !auto_mode_) {
    integral_ = Clamp(last_output, integral_limits_);
    proportional_ = 0.0;
    derivative_ = 0.0;
    has_last_input_ = false;
  }
  auto_mode_ = enabled;
}

std::tuple<double, double, double> Pid::components() const noexcept {
  return {proportional_, integral_, derivative_};
}

double Pid::Clamp(double value, const PidLimits& limits) noexcept {
  if (limits.min.has_value() && value < *limits.min) {
    value = *limits.min;
  }
  if (limits.max.has_value() && value > *limits.max) {
    value = *limits.max;
  }
  return value;
}

}  // namespace robot::control
