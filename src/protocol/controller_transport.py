import struct

import utime

from src.protocol.controller_snapshot import ControllerSnapshotV1

FRAME_MAGIC = b"\xA5\x5A"
FRAME_VERSION = 0x01
FRAME_TYPE_SNAPSHOT = 0x01
SNAPSHOT_FLAG_CONNECTED = 0x01
SNAPSHOT_PAYLOAD_LEN = 26
FRAME_HEADER_LEN = 6
FRAME_CRC_LEN = 2
FRAME_LEN = FRAME_HEADER_LEN + SNAPSHOT_PAYLOAD_LEN + FRAME_CRC_LEN

_PAYLOAD_FORMAT = "<IIBBHBBhhhhHH"


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def decode_snapshot_payload(payload, received_at_ms=None):
    if len(payload) != SNAPSHOT_PAYLOAD_LEN:
        raise ValueError("unexpected_snapshot_payload_length")

    if received_at_ms is None:
        received_at_ms = utime.ticks_ms()

    unpacked = struct.unpack(_PAYLOAD_FORMAT, payload)
    return ControllerSnapshotV1(
        seq=unpacked[0],
        source_uptime_ms=unpacked[1],
        connected=(unpacked[2] & SNAPSHOT_FLAG_CONNECTED) != 0,
        dpad=unpacked[3],
        buttons=unpacked[4],
        misc_buttons=unpacked[5],
        left_stick_x=unpacked[7],
        left_stick_y=unpacked[8],
        right_stick_x=unpacked[9],
        right_stick_y=unpacked[10],
        left_trigger=unpacked[11],
        right_trigger=unpacked[12],
        received_at_ms=received_at_ms,
    )


class ControllerFrameParser:
    """Stateful framed parser with resync on the fixed magic bytes."""

    def __init__(self):
        self._buffer = bytearray()

    def _drop_prefix(self, count):
        if count <= 0:
            return
        self._buffer = self._buffer[count:]

    def feed(self, chunk, received_at_ms=None):
        snapshots = []
        if not chunk:
            return snapshots

        self._buffer.extend(chunk)

        while True:
            if len(self._buffer) < FRAME_LEN:
                return snapshots

            if self._buffer[0] != FRAME_MAGIC[0] or self._buffer[1] != FRAME_MAGIC[1]:
                self._drop_prefix(1)
                continue

            version = self._buffer[2]
            message_type = self._buffer[3]
            payload_len = self._buffer[4] | (self._buffer[5] << 8)

            if version != FRAME_VERSION or message_type != FRAME_TYPE_SNAPSHOT:
                self._drop_prefix(1)
                continue

            if payload_len != SNAPSHOT_PAYLOAD_LEN:
                self._drop_prefix(1)
                continue

            frame = bytes(self._buffer[:FRAME_LEN])
            payload = frame[FRAME_HEADER_LEN:FRAME_HEADER_LEN + SNAPSHOT_PAYLOAD_LEN]
            expected_crc = frame[-2] | (frame[-1] << 8)
            actual_crc = crc16_ccitt(frame[2:-2])
            if actual_crc != expected_crc:
                self._drop_prefix(1)
                continue

            # A repeated seq is still a valid fresh frame in this transport.
            # ESP32 uses source_uptime_ms as the heartbeat/freshness signal
            # when the controller state itself has not changed.
            snapshots.append(decode_snapshot_payload(payload, received_at_ms=received_at_ms))
            self._drop_prefix(FRAME_LEN)
