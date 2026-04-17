#include "robot/hardware/motor_output_driver.hpp"

#include <cstdlib>

#include "robot/control/constants.hpp"

namespace robot::hardware {

AppliedMotorOutput NullMotorOutputDriver::Normalize(int signed_pwm) {
  const bool reverse = signed_pwm < 0;
  int pwm = std::abs(signed_pwm);
  if (pwm < robot::control::MotorLimits::kPwmMin) {
    pwm = 0;
  }
  if (pwm > robot::control::MotorLimits::kPwmMax) {
    pwm = robot::control::MotorLimits::kPwmMax;
  }
  return {
      .pwm_u16 = pwm,
      .reverse = reverse,
  };
}

void NullMotorOutputDriver::ApplySignedPwm(int left_pwm, int right_pwm) {
  last_output_.left = Normalize(left_pwm);
  last_output_.right = Normalize(right_pwm);
}

void NullMotorOutputDriver::Stop() {
  last_output_ = {};
}

}  // namespace robot::hardware
