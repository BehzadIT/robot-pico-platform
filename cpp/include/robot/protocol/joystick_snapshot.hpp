#pragma once

#include <cstdint>

namespace robot::protocol {

/**
 * Transport DTO for one joystick sample received from the ESP32 bridge.
 *
 * This type intentionally remains neutral. It captures validated transport
 * state but does not assign robot-driving meaning to the controls.
 */
struct JoystickSnapshot {
  uint32_t seq{0};
  uint32_t source_uptime_ms{0};
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
  uint32_t received_at_ms{0};
};

}  // namespace robot::protocol
