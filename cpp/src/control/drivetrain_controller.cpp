#include "robot/control/drivetrain_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>

#include "robot/control/constants.hpp"
#include "robot/support/logger.hpp"

namespace robot::control {

namespace {

constexpr double kPidKp = 0.489;
constexpr double kPidKi = 0.92;
constexpr double kPidKd = 0.0;
constexpr double kPidOutputMin = 0.0;
constexpr double kPidOutputMax = 1.0;
constexpr double kPidIntegralMin = 0.0;
constexpr double kPidIntegralMax = 1.0;
constexpr const char* kDrivetrainTag = "drivetrain";

}  // namespace

DrivetrainController::DrivetrainController(robot::hardware::DrivetrainEncoderSubsystem& encoders,
                                           robot::hardware::MotorOutputDriver& motor_driver,
                                           robot::hardware::Watchdog& watchdog)
    : encoders_(encoders),
      motor_driver_(motor_driver),
      watchdog_(watchdog),
      pid_left_(kPidKp, kPidKi, kPidKd, 0.0, {kPidOutputMin, kPidOutputMax},
                {kPidIntegralMin, kPidIntegralMax}),
      pid_right_(kPidKp, kPidKi, kPidKd, 0.0, {kPidOutputMin, kPidOutputMax},
                 {kPidIntegralMin, kPidIntegralMax}) {
  motor_driver_.Configure(MotorLimits::kPwmFreq);
}

DrivetrainCommandResult DrivetrainController::OnDriveAccepted() {
  if (state_ == DrivetrainState::kFaulted) {
    return {.ok = false, .code = "hf", .detail = "drivetrain faulted"};
  }
  if (state_ == DrivetrainState::kRecovering) {
    return {.ok = false, .code = "ri", .detail = "recovery in progress"};
  }
  if (state_ == DrivetrainState::kStopping) {
    return {.ok = false, .code = "si", .detail = "stop in progress"};
  }
  if (state_ != DrivetrainState::kRunning) {
    SetState(DrivetrainState::kStarting, "drive_requested");
    lifecycle_step_ = "drive_requested";
  }
  return {.ok = true, .code = "d", .detail = "drive_accepted"};
}

DrivetrainCommandResult DrivetrainController::RequestStop(std::optional<int> seq,
                                                          const std::string& reason) {
  pending_stop_seq_ = seq;
  pending_stop_reason_ = reason;

  if (state_ == DrivetrainState::kRecovering) {
    return {.ok = false, .code = "ri", .detail = "recovery in progress"};
  }

  if (state_ == DrivetrainState::kIdle || state_ == DrivetrainState::kFaulted) {
    pending_stop_seq_.reset();
    pending_stop_reason_ = "already_safe";
    lifecycle_step_ = "stop_already_safe";
    robot::support::LogI(kDrivetrainTag, "stop already safe reason=already_safe");
    return {
        .ok = true,
        .code = "s",
        .detail = "already_stopped",
        .completed = true,
    };
  }

  robot::support::LogI(kDrivetrainTag, std::string("stop requested reason=") + reason +
                                           " seq=" + std::to_string(seq.value_or(-1)));
  SetState(DrivetrainState::kStopping, reason);
  lifecycle_step_ = "stop_requested";
  return {.ok = true, .code = "s", .detail = "stop_requested"};
}

DrivetrainCommandResult DrivetrainController::RequestRecover(std::optional<int> seq) {
  if (state_ != DrivetrainState::kIdle && state_ != DrivetrainState::kFaulted) {
    robot::support::LogW(kDrivetrainTag, std::string("recover rejected state=") + StateName(state_) +
                                         " reason=unsafe_state");
    return {.ok = false, .code = "rc", .detail = "recovery requires idle or faulted state"};
  }
  pending_recover_seq_ = seq;
  robot::support::LogI(kDrivetrainTag,
                       std::string("recovery requested seq=") + std::to_string(seq.value_or(-1)));
  SetState(DrivetrainState::kRecovering, "recover_requested");
  lifecycle_step_ = "recover_requested";
  return {.ok = true, .code = "r", .detail = "recover_requested"};
}

std::optional<DrivetrainEvent> DrivetrainController::Tick(DriveCommandManager& drive_commands,
                                                          uint32_t now_ms) {
  const uint64_t tick_us = static_cast<uint64_t>(now_ms) * 1000ULL;

  switch (state_) {
    case DrivetrainState::kIdle:
    case DrivetrainState::kFaulted:
      FeedWatchdog();
      return std::nullopt;

    case DrivetrainState::kStopping:
      return StopCycle();

    case DrivetrainState::kRecovering:
      return RecoverCycle();

    case DrivetrainState::kStarting:
      if (!PrepareRunningState(tick_us)) {
        return LatchFault("encoder_init_failed", "encoder subsystem initialization failed");
      }
      EnsureWatchdog();
      FeedWatchdog();
      return std::nullopt;

    case DrivetrainState::kRunning:
      break;
  }

  if (const auto timeout =
          drive_commands.TimeoutStop(static_cast<int>(now_ms), SafetyTiming::kCommandTimeoutMs);
      timeout.has_value()) {
    last_command_seq_ = timeout->last_seq;
    const auto stop_result = RequestStop(std::nullopt, "command_timeout");
    if (!stop_result.ok) {
      return LatchFault("timeout_stop_failed", stop_result.detail);
    }
    return std::nullopt;
  }

  const DriveTarget drive_target = drive_commands.GetDriveTarget(static_cast<int>(now_ms));
  last_command_seq_ = drive_target.last_seq;
  if (drive_target.signed_target_rpm() == 0) {
    const auto stop_result = RequestStop(std::nullopt, "zero_target");
    if (!stop_result.ok) {
      return LatchFault("zero_target_stop_failed", stop_result.detail);
    }
    return std::nullopt;
  }

  const int target_sign = Sign(drive_target.signed_target_rpm());
  if (last_target_sign_ != 0 && target_sign != 0 && target_sign != last_target_sign_) {
    HandleDirectionReversal(tick_us, target_sign);
    FeedWatchdog();
    return std::nullopt;
  }

  if (hold_zero_until_us_.has_value() && tick_us < *hold_zero_until_us_) {
    FeedWatchdog();
    return std::nullopt;
  }
  hold_zero_until_us_.reset();

  try {
    auto measurement = encoders_.SampleVelocity(tick_us);
    measurement.command_age_ms = drive_target.command_age_ms;
    last_measurement_ = measurement;
    if (!measurement.valid) {
      FeedWatchdog();
      return std::nullopt;
    }

    const auto [left_pwm, right_pwm] = RunPid(drive_target, measurement);
    current_pwm_left_ = left_pwm;
    current_pwm_right_ = right_pwm;
    motor_driver_.ApplySignedPwm(left_pwm, right_pwm);
    last_target_sign_ = target_sign;
  } catch (const std::exception& exc) {
    return LatchFault("control_loop_failed", exc.what());
  }

  FeedWatchdog();
  return std::nullopt;
}

DrivetrainStatus DrivetrainController::status(const DriveTarget& drive_target) const {
  return {
      .state = state_,
      .fault = fault_,
      .current_pwm_left = current_pwm_left_,
      .current_pwm_right = current_pwm_right_,
      .last_command_seq = last_command_seq_,
      .drive_target = drive_target,
      .measurement = last_measurement_,
  };
}

int DrivetrainController::Clamp(int value, int min_value, int max_value) noexcept {
  return std::clamp(value, min_value, max_value);
}

int DrivetrainController::Sign(int value) noexcept {
  if (value > 0) {
    return 1;
  }
  if (value < 0) {
    return -1;
  }
  return 0;
}

const char* DrivetrainController::StateName(DrivetrainState state) noexcept {
  switch (state) {
    case DrivetrainState::kIdle:
      return "idle";
    case DrivetrainState::kStarting:
      return "starting";
    case DrivetrainState::kRunning:
      return "running";
    case DrivetrainState::kStopping:
      return "stopping";
    case DrivetrainState::kRecovering:
      return "recovering";
    case DrivetrainState::kFaulted:
      return "faulted";
  }
  return "unknown";
}

void DrivetrainController::SetState(DrivetrainState new_state, std::string_view reason) {
  if (state_ == new_state) {
    return;
  }
  const char* old_state = StateName(state_);
  state_ = new_state;
  const char* new_state_name = StateName(new_state);
  char buffer[160];
  std::snprintf(buffer, sizeof(buffer), "state transition old_state=%s new_state=%s reason=%.*s",
                old_state, new_state_name, static_cast<int>(reason.size()), reason.data());
  robot::support::LogI(kDrivetrainTag, buffer);
}

bool DrivetrainController::PrepareRunningState(uint64_t tick_us) {
  lifecycle_step_ = "starting_prepare";
  if (!encoders_.Initialize()) {
    return false;
  }
  (void)encoders_.PrimeVelocityEstimator(tick_us);
  ResetPidState("start");
  SetState(DrivetrainState::kRunning, "encoder_initialized");
  lifecycle_step_ = "running";
  return true;
}

std::optional<DrivetrainEvent> DrivetrainController::StopCycle() {
  lifecycle_step_ = "stop_cycle_enter";
  motor_driver_.Stop();
  current_pwm_left_ = 0;
  current_pwm_right_ = 0;
  ResetPidState("stop");
  last_measurement_ = {};
  const auto completed_seq = pending_stop_seq_;
  pending_stop_seq_.reset();
  pending_stop_reason_ = "manual_stop";
  SetState(DrivetrainState::kIdle, "stop_completed");
  lifecycle_step_ = "stop_cycle_complete";
  ++completed_stop_cycles_;
  EnsureWatchdog();
  FeedWatchdog();
  return DrivetrainEvent{
      .kind = DrivetrainEvent::Kind::kStopCompleted,
      .ok = true,
      .code = "s",
      .detail = "stop_completed",
      .seq = completed_seq,
  };
}

std::optional<DrivetrainEvent> DrivetrainController::RecoverCycle() {
  lifecycle_step_ = "recover_cycle_enter";
  motor_driver_.Stop();
  current_pwm_left_ = 0;
  current_pwm_right_ = 0;
  ResetPidState("recover");
  last_measurement_ = {};

  if (!encoders_.Recover()) {
    const auto fault_detail =
        encoders_.fault_info().has_value() ? encoders_.fault_info()->message : "encoder recovery failed";
    return LatchFault("recovery_failed", fault_detail);
  }

  const auto completed_seq = pending_recover_seq_;
  pending_recover_seq_.reset();
  if (fault_.has_value()) {
    robot::support::LogI(kDrivetrainTag,
                         std::string("clearing active drivetrain fault reason=recover_completed code=") +
                             fault_->code + " detail=" + fault_->detail);
  }
  fault_.reset();
  SetState(DrivetrainState::kIdle, "recover_completed");
  lifecycle_step_ = "recover_cycle_complete";
  FeedWatchdog();
  robot::support::LogI(kDrivetrainTag,
                       std::string("recovered seq=") + std::to_string(completed_seq.value_or(-1)));
  return DrivetrainEvent{
      .kind = DrivetrainEvent::Kind::kRecoverCompleted,
      .ok = true,
      .code = "r",
      .detail = "recover_completed",
      .seq = completed_seq,
  };
}

std::optional<DrivetrainEvent> DrivetrainController::LatchFault(std::string code,
                                                                std::string detail) {
  fault_ = {
      .code = std::move(code),
      .detail = std::move(detail),
      .lifecycle_step = lifecycle_step_,
  };
  last_fault_ = fault_;
  SetState(DrivetrainState::kFaulted, fault_->code);
  motor_driver_.Stop();
  current_pwm_left_ = 0;
  current_pwm_right_ = 0;
  ResetPidState("fault");
  robot::support::LogE(kDrivetrainTag, std::string("fault latched code=") + fault_->code +
                                           " detail=" + fault_->detail +
                                           " lifecycle_step=" + fault_->lifecycle_step);
  return DrivetrainEvent{
      .kind = DrivetrainEvent::Kind::kFaulted,
      .ok = false,
      .code = fault_->code,
      .detail = fault_->detail,
      .seq = std::nullopt,
  };
}

void DrivetrainController::ResetPidState(const char* reason) noexcept {
  pid_left_.Reset();
  pid_right_.Reset();
  hold_zero_until_us_.reset();
  last_target_sign_ = 0;
  robot::support::LogD(kDrivetrainTag, std::string("pid reset reason=") + reason);
}

void DrivetrainController::EnsureWatchdog() {
  if (watchdog_.armed()) {
    return;
  }
  if (!robot::config::kDrivetrainConfig.enable_watchdog) {
    if (!watchdog_disabled_logged_) {
      watchdog_disabled_logged_ = true;
      robot::support::LogI(kDrivetrainTag, "drivetrain watchdog disabled");
    }
    return;
  }
  if (robot::config::kDrivetrainConfig.watchdog_arm_after_first_stop &&
      completed_stop_cycles_ < 1) {
    if (!watchdog_pending_activation_logged_) {
      watchdog_pending_activation_logged_ = true;
      robot::support::LogI(kDrivetrainTag,
                           "drivetrain watchdog deferred until first clean stop cycle");
    }
    return;
  }
  if (!watchdog_.Arm(robot::config::kDrivetrainConfig.watchdog_timeout_ms)) {
    robot::support::LogW(kDrivetrainTag, "unable to enable drivetrain watchdog");
    return;
  }
  watchdog_pending_activation_logged_ = false;
  watchdog_disabled_logged_ = false;
  robot::support::LogI(
      kDrivetrainTag,
      std::string("drivetrain watchdog enabled (") +
          std::to_string(robot::config::kDrivetrainConfig.watchdog_timeout_ms) + "ms)");
}

void DrivetrainController::FeedWatchdog() {
  if (watchdog_.armed()) {
    watchdog_.Feed();
  }
}

void DrivetrainController::HandleDirectionReversal(uint64_t tick_us, int target_sign) {
  motor_driver_.Stop();
  current_pwm_left_ = 0;
  current_pwm_right_ = 0;
  ResetPidState("direction_reversal");
  (void)encoders_.PrimeVelocityEstimator(tick_us);
  hold_zero_until_us_ = tick_us + static_cast<uint64_t>(ControlTiming::kControlPeriodUs);
  last_target_sign_ = target_sign;
  robot::support::LogI(kDrivetrainTag,
                       std::string("direction reversal sign=") + std::to_string(target_sign) +
                           " hold_until_us=" + std::to_string(*hold_zero_until_us_));
}

std::pair<int, int> DrivetrainController::CalculateSetpoints(double norm_velocity,
                                                             int turning_angle) const {
  const double steering = std::clamp(
      turning_angle / static_cast<double>(SteeringLimits::kAngleMax), -1.0, 1.0);
  if (steering >= 0.0) {
    return {
        static_cast<int>(std::lround(norm_velocity * 10000.0)),
        static_cast<int>(std::lround(norm_velocity * (1.0 - steering) * 10000.0)),
    };
  }
  return {
      static_cast<int>(std::lround(norm_velocity * (1.0 + steering) * 10000.0)),
      static_cast<int>(std::lround(norm_velocity * 10000.0)),
  };
}

std::pair<int, int> DrivetrainController::RunPid(
    const DriveTarget& target,
    const robot::hardware::EncoderVelocitySample& sample) {
  const int signed_rpm = target.signed_target_rpm();
  const int direction = signed_rpm < 0 ? -1 : 1;
  const double abs_rpm = std::abs(static_cast<double>(signed_rpm));
  const double norm_velocity = NormalizeRpm(abs_rpm);
  const auto [left_set_scaled, right_set_scaled] =
      CalculateSetpoints(norm_velocity, target.target_angle);
  const double left_set = left_set_scaled / 10000.0;
  const double right_set = right_set_scaled / 10000.0;

  pid_left_.set_setpoint(left_set);
  pid_right_.set_setpoint(right_set);

  const double norm_measured_left = NormalizeRpm(std::abs(sample.left_rpm));
  const double norm_measured_right = NormalizeRpm(std::abs(sample.right_rpm));
  const double dt_seconds = sample.dt_us / 1'000'000.0;

  const int left_pwm =
      Clamp(DenormalizePwm(pid_left_.Update(norm_measured_left, dt_seconds)) * direction,
            -MotorLimits::kPwmMax, MotorLimits::kPwmMax);
  const int right_pwm =
      Clamp(DenormalizePwm(pid_right_.Update(norm_measured_right, dt_seconds)) * direction,
            -MotorLimits::kPwmMax, MotorLimits::kPwmMax);
  return {left_pwm, right_pwm};
}

int DrivetrainController::DenormalizePwm(double norm) {
  const int pwm_range = MotorLimits::kPwmMax - MotorLimits::kPwmMin;
  if (norm < 0.0) {
    return -static_cast<int>(std::lround(MotorLimits::kPwmMin + std::abs(norm) * pwm_range));
  }
  return static_cast<int>(std::lround(MotorLimits::kPwmMin + norm * pwm_range));
}

double DrivetrainController::NormalizeRpm(double rpm) {
  return rpm / static_cast<double>(MotorLimits::kRpmMax);
}

}  // namespace robot::control
