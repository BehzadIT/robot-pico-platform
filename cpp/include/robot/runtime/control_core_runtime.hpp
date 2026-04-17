#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "robot/control/controller_uart_pipeline.hpp"
#include "robot/control/drivetrain_controller.hpp"
#include "robot/control/drive_command_manager.hpp"
#include "robot/hardware/drivetrain_encoder_subsystem.hpp"
#include "robot/hardware/motor_output_driver.hpp"
#include "robot/hardware/watchdog.hpp"
#include "robot/runtime/shared_runtime_state.hpp"
#include "robot/support/logger.hpp"

namespace robot::runtime {

struct ControlCoreStatus {
  robot::control::DriveTarget drive_target{};
  robot::control::ControllerPipelineStatus controller{};
  robot::control::DrivetrainStatus drivetrain{};
};

/**
 * Control-core runtime owner.
 *
 * This object models the eventual Core 1 runtime: it owns command admission,
 * controller UART semantics, and the robot-level motion target seen by the
 * future drivetrain worker. Service-side code may enqueue requests, but only
 * this runtime may decide whether motion intent is accepted.
 */
class ControlCoreRuntime {
 public:
  explicit ControlCoreRuntime(SharedRuntimeState& shared_state);
  ControlCoreRuntime(SharedRuntimeState& shared_state,
                     robot::hardware::EncoderCounterFactory encoder_counter_factory,
                     std::unique_ptr<robot::hardware::MotorOutputDriver> motor_driver,
                     std::unique_ptr<robot::hardware::Watchdog> watchdog);

  void FeedControllerBytes(const uint8_t* chunk, std::size_t length, uint32_t now_ms);
  void Tick(uint32_t now_ms);

  [[nodiscard]] ControlCoreStatus status(uint32_t now_ms) const;

 private:
  void HandleTransportCommand(const TransportCommand& command, uint32_t now_ms);
  void HandleControllerPipeline(uint32_t now_ms);
  void HandleDrivetrainEvent(const robot::control::DrivetrainEvent& event);
  void PublishCompletion(bool ok,
                         std::string code,
                         std::string detail,
                         std::optional<int> seq);

  SharedRuntimeState& shared_state_;
  robot::control::DriveCommandManager drive_command_manager_;
  robot::control::ControllerUartPipeline controller_uart_pipeline_;
  std::unique_ptr<robot::hardware::MotorOutputDriver> owned_motor_driver_;
  std::unique_ptr<robot::hardware::Watchdog> owned_watchdog_;
  robot::hardware::MotorOutputDriver* motor_driver_;
  robot::hardware::Watchdog* watchdog_;
  robot::hardware::DrivetrainEncoderSubsystem encoder_subsystem_;
  robot::control::DrivetrainController drivetrain_controller_;
  robot::support::DistinctFilter last_controller_mode_;
};

}  // namespace robot::runtime
