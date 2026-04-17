#pragma once

#include <optional>
#include <tuple>

namespace robot::control {

struct PidLimits {
  std::optional<double> min;
  std::optional<double> max;
};

/**
 * Project-owned PID primitive for the deterministic drivetrain path.
 *
 * This mirrors the current MicroPython fork semantics that matter for parity:
 * explicit dt scaling, independent integral limits, and reset preserving
 * configured bounds.
 */
class Pid {
 public:
  Pid(double kp,
      double ki,
      double kd,
      double setpoint,
      PidLimits output_limits,
      PidLimits integral_limits);

  [[nodiscard]] double Update(double measurement, double dt_seconds);
  void Reset() noexcept;
  void SetAutoMode(bool enabled, double last_output = 0.0) noexcept;
  void set_setpoint(double setpoint) noexcept { setpoint_ = setpoint; }
  [[nodiscard]] double setpoint() const noexcept { return setpoint_; }

  [[nodiscard]] std::tuple<double, double, double> components() const noexcept;
  [[nodiscard]] bool auto_mode() const noexcept { return auto_mode_; }
  [[nodiscard]] PidLimits output_limits() const noexcept { return output_limits_; }
  [[nodiscard]] PidLimits integral_limits() const noexcept {
    return integral_limits_;
  }

 private:
  static double Clamp(double value, const PidLimits& limits) noexcept;

  double kp_;
  double ki_;
  double kd_;
  double setpoint_;
  PidLimits output_limits_;
  PidLimits integral_limits_;
  bool auto_mode_{true};
  bool has_last_input_{false};
  double last_input_{0.0};
  double proportional_{0.0};
  double integral_{0.0};
  double derivative_{0.0};
};

}  // namespace robot::control
