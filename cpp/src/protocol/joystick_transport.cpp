#include "robot/protocol/joystick_transport.hpp"

#include <algorithm>
#include <cstring>

namespace robot::protocol {

namespace {

constexpr uint8_t kFrameMagic0 = 0xA5;
constexpr uint8_t kFrameMagic1 = 0x5A;

uint16_t ReadU16Le(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(data[1] << 8);
}

uint32_t ReadU32Le(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int16_t ReadI16Le(const uint8_t* data) {
  return static_cast<int16_t>(ReadU16Le(data));
}

}  // namespace

uint16_t Crc16Ccitt(const uint8_t* data, std::size_t length) noexcept {
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i] << 8);
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
      } else {
        crc = static_cast<uint16_t>(crc << 1U);
      }
    }
  }
  return crc;
}

std::optional<JoystickSnapshot> DecodeJoystickSnapshotPayload(
    const uint8_t* payload,
    std::size_t length,
    uint32_t received_at_ms) noexcept {
  if (length != kSnapshotPayloadLen) {
    return std::nullopt;
  }

  JoystickSnapshot snapshot;
  snapshot.seq = ReadU32Le(payload + 0);
  snapshot.source_uptime_ms = ReadU32Le(payload + 4);
  snapshot.connected = (payload[8] & kSnapshotFlagConnected) != 0;
  snapshot.dpad = payload[9];
  snapshot.buttons = ReadU16Le(payload + 10);
  snapshot.misc_buttons = payload[12];
  snapshot.left_stick_x = ReadI16Le(payload + 14);
  snapshot.left_stick_y = ReadI16Le(payload + 16);
  snapshot.right_stick_x = ReadI16Le(payload + 18);
  snapshot.right_stick_y = ReadI16Le(payload + 20);
  snapshot.left_trigger = ReadU16Le(payload + 22);
  snapshot.right_trigger = ReadU16Le(payload + 24);
  snapshot.received_at_ms = received_at_ms;
  return snapshot;
}

std::vector<JoystickSnapshot> JoystickFrameParser::Feed(const uint8_t* chunk,
                                                        std::size_t length,
                                                        uint32_t received_at_ms) {
  std::vector<JoystickSnapshot> snapshots;
  if (chunk == nullptr || length == 0) {
    return snapshots;
  }

  buffer_.insert(buffer_.end(), chunk, chunk + length);

  while (buffer_.size() >= kFrameLen) {
    if (buffer_[0] != kFrameMagic0 || buffer_[1] != kFrameMagic1) {
      DropPrefix(1);
      continue;
    }

    const uint8_t version = buffer_[2];
    const uint8_t message_type = buffer_[3];
    const uint16_t payload_len = ReadU16Le(buffer_.data() + 4);

    if (version != kFrameVersion || message_type != kFrameTypeSnapshot ||
        payload_len != kSnapshotPayloadLen) {
      DropPrefix(1);
      continue;
    }

    const uint16_t expected_crc =
        ReadU16Le(buffer_.data() + (kFrameLen - kFrameCrcLen));
    const uint16_t actual_crc =
        Crc16Ccitt(buffer_.data() + 2, kFrameLen - 4);
    if (actual_crc != expected_crc) {
      DropPrefix(1);
      continue;
    }

    const auto decoded = DecodeJoystickSnapshotPayload(
        buffer_.data() + kFrameHeaderLen, kSnapshotPayloadLen, received_at_ms);
    if (decoded.has_value()) {
      snapshots.push_back(*decoded);
    }
    DropPrefix(kFrameLen);
  }

  return snapshots;
}

void JoystickFrameParser::DropPrefix(std::size_t count) {
  count = std::min(count, buffer_.size());
  buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(count));
}

}  // namespace robot::protocol
