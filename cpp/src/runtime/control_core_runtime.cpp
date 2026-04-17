#include "robot/runtime/control_core_runtime.hpp"

#include <memory>

#include "robot/config/config.hpp"
#include "robot/control/constants.hpp"
#include "robot/hardware/encoder_counter.hpp"
#include "robot/support/logger.hpp"

namespace robot::runtime {

namespace {

constexpr const char* kControllerTag = "controller_drive";

class StubEncoderCounter final : public robot::hardware::EncoderCounter {
 public:
  [[nodiscard]] int32_t Read() const override { return count_; }
  void SetZero() override { count_ = 0; }
  void Deinit() override {}
  void SetCount(int32_t count) { count_ = count; }

 private:
  int32_t count_{0};
};

robot::hardware::EncoderCounterFactory MakeStubCounterFactory() {
  return [](const robot::hardware::EncoderCounterConfig&) {
    return std::make_unique<StubEncoderCounter>();
  };
}

}  // namespace

ControlCoreRuntime::ControlCoreRuntime(SharedRuntimeState& shared_state)
    : ControlCoreRuntime(shared_state, MakeStubCounterFactory(),
                         std::make_unique<robot::hardware::NullMotorOutputDriver>(),
                         std::make_unique<robot::hardware::NullWatchdog>()) {}

ControlCoreRuntime::ControlCoreRuntime(
    SharedRuntimeState& shared_state,
    robot::hardware::EncoderCounterFactory encoder_counter_factory,
    std::unique_ptr<robot::hardware::MotorOutputDriver> motor_driver,
    std::unique_ptr<robot::hardware::Watchdog> watchdog)
    : shared_state_(shared_state),
      drive_command_manager_(robot::control::SafetyTiming::kTimeoutRestartGuardMs),
      controller_uart_pipeline_({
          .publish_period_ms =
              static_cast<uint32_t>(robot::config::kControllerDriveConfig.publish_period_ms),
          .snapshot_stale_ms =
              static_cast<uint32_t>(robot::config::kControllerDriveConfig.snapshot_stale_ms),
          .trigger_press_threshold =
              robot::config::kControllerDriveConfig.trigger_press_threshold,
          .trigger_release_threshold =
              robot::config::kControllerDriveConfig.trigger_release_threshold,
          .turn_deadzone = robot::config::kControllerDriveConfig.turn_deadzone,
      }),
      owned_motor_driver_(std::move(motor_driver)),
      owned_watchdog_(std::move(watchdog)),
      motor_driver_(owned_motor_driver_.get()),
      watchdog_(owned_watchdog_.get()),
      encoder_subsystem_(std::move(encoder_counter_factory)),
      drivetrain_controller_(encoder_subsystem_, *motor_driver_, *watchdog_) {}

void ControlCoreRuntime::FeedControllerBytes(const uint8_t* chunk,
                                             std::size_t length,
                                             uint32_t now_ms) {
  controller_uart_pipeline_.FeedBytes(chunk, length, now_ms);
}

void ControlCoreRuntime::Tick(uint32_t now_ms) {
  if (const auto command = shared_state_.inbound_commands.Pop(); command.has_value()) {
    HandleTransportCommand(*command, now_ms);
  }

  HandleControllerPipeline(now_ms);

  if (const auto drivetrain_event =
          drivetrain_controller_.Tick(drive_command_manager_, now_ms);
      drivetrain_event.has_value()) {
    HandleDrivetrainEvent(*drivetrain_event);
  }
}

ControlCoreStatus ControlCoreRuntime::status(uint32_t now_ms) const {
  const auto drive_target = drive_command_manager_.GetDriveTarget(static_cast<int>(now_ms));
  return {
      .drive_target = drive_target,
      .controller = controller_uart_pipeline_.status(),
      .drivetrain = drivetrain_controller_.status(drive_target),
  };
}

void ControlCoreRuntime::HandleTransportCommand(const TransportCommand& command,
                                                uint32_t now_ms) {
  switch (command.kind) {
    case TransportCommandKind::kDrive: {
      const auto result = drive_command_manager_.AcceptDriveRequest(
          command.drive_request, static_cast<int>(now_ms), command.command_source_id);
      if (result.ok) {
        const auto controller_result = drivetrain_controller_.OnDriveAccepted();
        if (!controller_result.ok) {
          robot::support::LogW(
              "ws", std::string("rejecting drive while drivetrain code=") + controller_result.code +
                        " detail=" + controller_result.detail);
          (void)drive_command_manager_.RevokeDriveRequest(static_cast<int>(now_ms), command.seq,
                                                          command.command_source_id);
          PublishCompletion(false, controller_result.code, controller_result.detail,
                            command.seq);
          break;
        }
      }
      PublishCompletion(result.ok, result.code, result.detail, command.seq);
      break;
    }
    case TransportCommandKind::kStop: {
      const bool stop_owned = drive_command_manager_.Stop(static_cast<int>(now_ms),
                                                          command.command_source_id);
      if (!stop_owned) {
        PublishCompletion(false, "ic", "invalid command source", command.seq);
        break;
      }
      const auto result = drivetrain_controller_.RequestStop(command.seq, "manual_stop");
      if (result.completed) {
        PublishCompletion(result.ok, result.code, result.detail, command.seq);
      } else if (!result.ok) {
        PublishCompletion(false, result.code, result.detail, command.seq);
      }
      break;
    }
    case TransportCommandKind::kRecover: {
      const auto result = drivetrain_controller_.RequestRecover(command.seq);
      if (result.completed || !result.ok) {
        PublishCompletion(result.ok, result.code, result.detail, command.seq);
      }
      break;
    }
    case TransportCommandKind::kDisconnect: {
      if (drive_command_manager_.ClearCommandSourceIfActive(command.command_source_id,
                                                            static_cast<int>(now_ms))) {
        (void)drivetrain_controller_.RequestStop(std::nullopt, "disconnect_stop");
      }
      break;
    }
  }
}

void ControlCoreRuntime::HandleControllerPipeline(uint32_t now_ms) {
  const auto command = controller_uart_pipeline_.Tick(now_ms);
  const auto current_status = controller_uart_pipeline_.status();
  if (last_controller_mode_.ShouldEmit(current_status.mode)) {
    robot::support::LogI(
        kControllerTag,
        std::string("controller mode=") + current_status.mode + " reason=" +
            current_status.last_stop_reason + " age_ms=" +
            std::to_string(current_status.last_age_ms) + " snapshot_seq=" +
            std::to_string(current_status.last_snapshot_seq));
  }
  if (!command.has_value()) {
    return;
  }

  if (command->kind == robot::control::ControllerCommandKind::kDrive) {
    const auto accepted = drive_command_manager_.AcceptDriveRequest(
        command->drive_request, static_cast<int>(now_ms), std::string("controller-uart"));
    if (accepted.ok) {
      const auto controller_result = drivetrain_controller_.OnDriveAccepted();
      if (!controller_result.ok) {
        robot::support::LogW(
            kControllerTag, std::string("controller drive rejected code=") +
                                controller_result.code + " detail=" + controller_result.detail);
        (void)drive_command_manager_.RevokeDriveRequest(static_cast<int>(now_ms),
                                                        command->drive_request.seq,
                                                        std::string("controller-uart"));
      }
    } else {
      robot::support::LogW(
          kControllerTag, std::string("controller drive rejected code=") + accepted.code +
                              " detail=" + accepted.detail);
    }
    return;
  }

  if (command->kind == robot::control::ControllerCommandKind::kStop) {
    const bool stopped = drive_command_manager_.Stop(static_cast<int>(now_ms),
                                                     std::string("controller-uart"));
    if (stopped) {
      robot::support::LogI(kControllerTag,
                           std::string("controller stop requested reason=") + command->reason);
      (void)drivetrain_controller_.RequestStop(std::nullopt, command->reason);
    }
  }
}

void ControlCoreRuntime::HandleDrivetrainEvent(
    const robot::control::DrivetrainEvent& event) {
  PublishCompletion(event.ok, event.code, event.detail, event.seq);
}

void ControlCoreRuntime::PublishCompletion(bool ok,
                                           std::string code,
                                           std::string detail,
                                           std::optional<int> seq) {
  const bool queued = shared_state_.outbound_completions.Push(ControlCompletion{
      .ok = ok,
      .code = std::move(code),
      .detail = std::move(detail),
      .seq = seq,
  });
  (void)queued;
}

}  // namespace robot::runtime
