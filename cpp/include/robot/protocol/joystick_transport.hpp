#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "robot/protocol/joystick_snapshot.hpp"

namespace robot::protocol {

inline constexpr uint8_t kFrameVersion = 0x01;
inline constexpr uint8_t kFrameTypeSnapshot = 0x01;
inline constexpr uint8_t kSnapshotFlagConnected = 0x01;
inline constexpr std::size_t kSnapshotPayloadLen = 26;
inline constexpr std::size_t kFrameHeaderLen = 6;
inline constexpr std::size_t kFrameCrcLen = 2;
inline constexpr std::size_t kFrameLen =
    kFrameHeaderLen + kSnapshotPayloadLen + kFrameCrcLen;

uint16_t Crc16Ccitt(const uint8_t* data, std::size_t length) noexcept;

std::optional<JoystickSnapshot> DecodeJoystickSnapshotPayload(
    const uint8_t* payload,
    std::size_t length,
    uint32_t received_at_ms) noexcept;

/**
 * Byte-stream parser for joystick UART frames.
 *
 * This type owns framing, resynchronization, version checks, and CRC
 * validation only. It must not assign control meaning to the decoded data.
 */
class JoystickFrameParser {
 public:
  [[nodiscard]] std::vector<JoystickSnapshot> Feed(const uint8_t* chunk,
                                                   std::size_t length,
                                                   uint32_t received_at_ms);

 private:
  void DropPrefix(std::size_t count);

  std::vector<uint8_t> buffer_;
};

}  // namespace robot::protocol
