#pragma once

namespace robot::hardware {

struct AppliedMotorOutput {
  int pwm_u16{0};
  bool reverse{false};
};

struct DifferentialMotorOutput {
  AppliedMotorOutput left{};
  AppliedMotorOutput right{};
};

/**
 * Signed PWM/DIR output boundary.
 *
 * Control code computes signed PWM values. Hardware code alone translates
 * those into the Cytron PWM/DIR pins, preserving single-writer ownership.
 */
class MotorOutputDriver {
 public:
  virtual ~MotorOutputDriver() = default;

  virtual void Configure(int pwm_frequency_hz) = 0;
  virtual void ApplySignedPwm(int left_pwm, int right_pwm) = 0;
  virtual void Stop() = 0;

  [[nodiscard]] virtual DifferentialMotorOutput last_output() const = 0;
};

/**
 * Host-safe no-op driver used until Pico hardware is present.
 */
class NullMotorOutputDriver final : public MotorOutputDriver {
 public:
  void Configure(int /*pwm_frequency_hz*/) override {}
  void ApplySignedPwm(int left_pwm, int right_pwm) override;
  void Stop() override;
  [[nodiscard]] DifferentialMotorOutput last_output() const override {
    return last_output_;
  }

 private:
  static AppliedMotorOutput Normalize(int signed_pwm);

  DifferentialMotorOutput last_output_{};
};

}  // namespace robot::hardware
