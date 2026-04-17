#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "robot/config/config.hpp"
#include "robot/control/drive_command_manager.hpp"
#include "robot/control/pid.hpp"
#include "robot/hardware/drivetrain_encoder_subsystem.hpp"
#include "robot/hardware/motor_output_driver.hpp"
#include "robot/hardware/watchdog.hpp"

namespace robot::control {

enum class DrivetrainState {
  kIdle,
  kStarting,
  kRunning,
  kStopping,
  kRecovering,
  kFaulted,
};

struct DrivetrainFault {
  std::string code;
  std::string detail;
  std::string lifecycle_step;
};

struct DrivetrainStatus {
  DrivetrainState state{DrivetrainState::kIdle};
  std::optional<DrivetrainFault> fault;
  int current_pwm_left{0};
  int current_pwm_right{0};
  int last_command_seq{-1};
  DriveTarget drive_target{};
  robot::hardware::EncoderVelocitySample measurement{};
};

struct DrivetrainCommandResult {
  bool ok{false};
  std::string code;
  std::string detail;
  bool completed{false};
};

struct DrivetrainEvent {
  enum class Kind {
    kStopCompleted,
    kRecoverCompleted,
    kFaulted,
  };

  Kind kind{Kind::kStopCompleted};
  bool ok{false};
  std::string code;
  std::string detail;
  std::optional<int> seq;
};

/**
 * Deterministic drivetrain owner for the control core.
 *
 * This preserves the project’s non-RTOS boundary: control code owns accepted
 * motion intent, fixed-cadence loop semantics, encoder truth, safe stop,
 * recovery, and watchdog progress. Service-side code may request actions but
 * must not directly mutate actuator state.
 */
class DrivetrainController {
 public:
  DrivetrainController(robot::hardware::DrivetrainEncoderSubsystem& encoders,
                       robot::hardware::MotorOutputDriver& motor_driver,
                       robot::hardware::Watchdog& watchdog);

  [[nodiscard]] DrivetrainCommandResult OnDriveAccepted();
  [[nodiscard]] DrivetrainCommandResult RequestStop(std::optional<int> seq,
                                                    const std::string& reason);
  [[nodiscard]] DrivetrainCommandResult RequestRecover(std::optional<int> seq);
  [[nodiscard]] std::optional<DrivetrainEvent> Tick(DriveCommandManager& drive_commands,
                                                    uint32_t now_ms);

  [[nodiscard]] DrivetrainStatus status(const DriveTarget& drive_target) const;
  [[nodiscard]] DrivetrainState state() const noexcept { return state_; }

 private:
  static int Clamp(int value, int min_value, int max_value) noexcept;
  static int Sign(int value) noexcept;
  static const char* StateName(DrivetrainState state) noexcept;

  void SetState(DrivetrainState new_state, std::string_view reason);
  [[nodiscard]] bool PrepareRunningState(uint64_t tick_us);
  [[nodiscard]] std::optional<DrivetrainEvent> StopCycle();
  [[nodiscard]] std::optional<DrivetrainEvent> RecoverCycle();
  [[nodiscard]] std::optional<DrivetrainEvent> LatchFault(std::string code,
                                                          std::string detail);
  void ResetPidState(const char* reason) noexcept;
  void EnsureWatchdog();
  void FeedWatchdog();
  void HandleDirectionReversal(uint64_t tick_us, int target_sign);
  [[nodiscard]] std::pair<int, int> CalculateSetpoints(double norm_velocity,
                                                       int turning_angle) const;
  [[nodiscard]] std::pair<int, int> RunPid(const DriveTarget& target,
                                           const robot::hardware::EncoderVelocitySample& sample);
  static int DenormalizePwm(double norm);
  static double NormalizeRpm(double rpm);

  robot::hardware::DrivetrainEncoderSubsystem& encoders_;
  robot::hardware::MotorOutputDriver& motor_driver_;
  robot::hardware::Watchdog& watchdog_;
  Pid pid_left_;
  Pid pid_right_;
  DrivetrainState state_{DrivetrainState::kIdle};
  std::optional<DrivetrainFault> fault_;
  std::optional<DrivetrainFault> last_fault_;
  std::string lifecycle_step_{"init"};
  int current_pwm_left_{0};
  int current_pwm_right_{0};
  std::optional<uint64_t> hold_zero_until_us_;
  int last_target_sign_{0};
  int completed_stop_cycles_{0};
  bool watchdog_pending_activation_logged_{false};
  bool watchdog_disabled_logged_{false};
  bool summary_dirty_{false};
  int last_command_seq_{-1};
  std::optional<int> pending_stop_seq_;
  std::string pending_stop_reason_{"manual_stop"};
  std::optional<int> pending_recover_seq_;
  robot::hardware::EncoderVelocitySample last_measurement_{};
};

}  // namespace robot::control
