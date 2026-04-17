#include "robot/hardware/pico_motor_output_driver.hpp"

#include <cmath>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "robot/control/constants.hpp"

namespace robot::hardware {

namespace {

void ConfigureSlice(int gpio, int pwm_frequency_hz) {
  gpio_set_function(gpio, GPIO_FUNC_PWM);
  const uint slice = pwm_gpio_to_slice_num(gpio);
  pwm_config config = pwm_get_default_config();
  pwm_config_set_wrap(&config, robot::control::MotorLimits::kPwmMax);
  const float clkdiv = static_cast<float>(clock_get_hz(clk_sys)) /
                       (pwm_frequency_hz * (robot::control::MotorLimits::kPwmMax + 1.0f));
  pwm_config_set_clkdiv(&config, clkdiv > 1.0f ? clkdiv : 1.0f);
  pwm_init(slice, &config, true);
}

}  // namespace

AppliedMotorOutput PicoMotorOutputDriver::Normalize(int signed_pwm) {
  const bool reverse = signed_pwm < 0;
  int pwm = std::abs(signed_pwm);
  if (pwm < robot::control::MotorLimits::kPwmMin) {
    pwm = 0;
  }
  if (pwm > robot::control::MotorLimits::kPwmMax) {
    pwm = robot::control::MotorLimits::kPwmMax;
  }
  return {.pwm_u16 = pwm, .reverse = reverse};
}

void PicoMotorOutputDriver::Configure(int pwm_frequency_hz) {
  gpio_init(robot::control::MotorPins::kLeftDir);
  gpio_init(robot::control::MotorPins::kRightDir);
  gpio_set_dir(robot::control::MotorPins::kLeftDir, GPIO_OUT);
  gpio_set_dir(robot::control::MotorPins::kRightDir, GPIO_OUT);
  ConfigureSlice(robot::control::MotorPins::kLeftPwm, pwm_frequency_hz);
  ConfigureSlice(robot::control::MotorPins::kRightPwm, pwm_frequency_hz);
  configured_ = true;
  Stop();
}

void PicoMotorOutputDriver::ApplyOne(int pwm_pin,
                                     int dir_pin,
                                     int signed_pwm,
                                     AppliedMotorOutput* state) {
  *state = Normalize(signed_pwm);
  gpio_put(dir_pin, state->reverse ? 1 : 0);
  pwm_set_gpio_level(pwm_pin, state->pwm_u16);
}

void PicoMotorOutputDriver::ApplySignedPwm(int left_pwm, int right_pwm) {
  if (!configured_) {
    Configure(robot::control::MotorLimits::kPwmFreq);
  }
  ApplyOne(robot::control::MotorPins::kLeftPwm, robot::control::MotorPins::kLeftDir,
           left_pwm, &last_output_.left);
  ApplyOne(robot::control::MotorPins::kRightPwm, robot::control::MotorPins::kRightDir,
           right_pwm, &last_output_.right);
}

void PicoMotorOutputDriver::Stop() {
  if (!configured_) {
    return;
  }
  gpio_put(robot::control::MotorPins::kLeftDir, 0);
  gpio_put(robot::control::MotorPins::kRightDir, 0);
  pwm_set_gpio_level(robot::control::MotorPins::kLeftPwm, 0);
  pwm_set_gpio_level(robot::control::MotorPins::kRightPwm, 0);
  last_output_ = {};
}

}  // namespace robot::hardware
