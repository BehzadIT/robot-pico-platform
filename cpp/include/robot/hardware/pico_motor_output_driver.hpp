#pragma once

#include "robot/hardware/motor_output_driver.hpp"

namespace robot::hardware {

class PicoMotorOutputDriver final : public MotorOutputDriver {
 public:
  void Configure(int pwm_frequency_hz) override;
  void ApplySignedPwm(int left_pwm, int right_pwm) override;
  void Stop() override;
  [[nodiscard]] DifferentialMotorOutput last_output() const override {
    return last_output_;
  }

 private:
  static AppliedMotorOutput Normalize(int signed_pwm);
  void ApplyOne(int pwm_pin, int dir_pin, int signed_pwm, AppliedMotorOutput* state);

  bool configured_{false};
  DifferentialMotorOutput last_output_{};
};

}  // namespace robot::hardware
