#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "robot/control/controller_drive_mapper.hpp"
#include "robot/protocol/api_drive_request.hpp"
#include "robot/protocol/joystick_snapshot.hpp"
#include "robot/protocol/joystick_transport.hpp"

namespace robot::control {

enum class ControllerCommandKind {
  kNone,
  kDrive,
  kStop,
};

struct ControllerCommand {
  ControllerCommandKind kind{ControllerCommandKind::kNone};
  robot::protocol::ApiDriveRequest drive_request{};
  std::string reason;
  std::string mode;
  uint32_t snapshot_seq{0};
  int age_ms{0};
};

struct ControllerPipelineStatus {
  bool has_snapshot{false};
  bool driving{false};
  std::string mode{"inactive"};
  std::string last_stop_reason;
  uint32_t last_snapshot_seq{0};
  uint32_t last_source_uptime_ms{0};
  uint32_t last_receive_ms{0};
  bool connected{false};
  uint8_t dpad{0};
  uint16_t buttons{0};
  uint8_t misc_buttons{0};
  int16_t left_stick_x{0};
  int16_t left_stick_y{0};
  int16_t right_stick_x{0};
  int16_t right_stick_y{0};
  uint16_t left_trigger{0};
  uint16_t right_trigger{0};
  int last_throttle_percent{0};
  int last_turn_percent{0};
  int last_age_ms{0};
  uint32_t frames_ok{0};
};

/**
 * Own the live controller UART path on the control core.
 *
 * This preserves the project boundary learned from the MicroPython runtime:
 * if controller ingress can directly influence motion, its freshness,
 * robot-specific mapping, republish cadence, and controller-triggered stop
 * intent should stay with the control owner rather than being spread across
 * service tasks.
 */
class ControllerUartPipeline {
 public:
  explicit ControllerUartPipeline(ControllerDriveConfig config);

  void FeedBytes(const uint8_t* chunk, std::size_t length, uint32_t now_ms);

  [[nodiscard]] std::optional<ControllerCommand> Tick(uint32_t now_ms);
  [[nodiscard]] ControllerPipelineStatus status() const;

 private:
  [[nodiscard]] int NextDriveSeq(uint32_t snapshot_seq, uint32_t now_ms) noexcept;

  ControllerDriveConfig config_;
  ControllerDriveMapper mapper_;
  robot::protocol::JoystickFrameParser parser_;
  std::optional<robot::protocol::JoystickSnapshot> latest_snapshot_;
  std::string last_mode_{"inactive"};
  std::string last_stop_reason_;
  bool driving_{false};
  uint32_t last_publish_ms_{0};
  int last_emitted_seq_{0};
  uint32_t frames_ok_{0};
  int last_throttle_percent_{0};
  int last_turn_percent_{0};
  int last_age_ms_{0};
};

}  // namespace robot::control
