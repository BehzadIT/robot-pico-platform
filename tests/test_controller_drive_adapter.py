import unittest

import utime

from src.control.controller_input_mapping import ControllerDriveMapper
from src.protocol.joystick_snapshot import JoystickSnapshot


TEST_CONFIG = {
    "snapshot_stale_ms": 250,
    "trigger_press_threshold": 120,
    "trigger_release_threshold": 80,
    "turn_deadzone": 240,
}


def make_snapshot(
    connected=True,
    left_trigger=0,
    right_trigger=0,
    left_stick_x=0,
    seq=1,
    received_at_ms=0,
):
    return JoystickSnapshot(
        seq=seq,
        source_uptime_ms=received_at_ms,
        connected=connected,
        dpad=0,
        buttons=0,
        misc_buttons=0,
        left_stick_x=left_stick_x,
        left_stick_y=0,
        right_stick_x=0,
        right_stick_y=0,
        left_trigger=left_trigger,
        right_trigger=right_trigger,
        received_at_ms=received_at_ms,
    )


class ControllerDriveMapperTest(unittest.TestCase):
    def setUp(self):
        utime.reset()
        self.mapper = ControllerDriveMapper(TEST_CONFIG)

    def test_right_trigger_maps_to_forward_drive(self):
        snapshot = make_snapshot(right_trigger=560)
        mapped = self.mapper.map_snapshot(snapshot, now_ms=0)

        self.assertEqual(mapped["mode"], "driving")
        self.assertGreater(mapped["throttle_percent"], 0)

    def test_left_trigger_maps_to_reverse_drive(self):
        snapshot = make_snapshot(left_trigger=560)
        mapped = self.mapper.map_snapshot(snapshot, now_ms=0)

        self.assertEqual(mapped["mode"], "driving")
        self.assertLess(mapped["throttle_percent"], 0)

    def test_equal_triggers_return_neutral(self):
        snapshot = make_snapshot(left_trigger=400, right_trigger=400)
        mapped = self.mapper.map_snapshot(snapshot, now_ms=0)

        self.assertEqual(mapped["mode"], "armed_neutral")
        self.assertEqual(mapped["throttle_percent"], 0)

    def test_trigger_hysteresis_uses_press_and_release_thresholds(self):
        active_snapshot = make_snapshot(right_trigger=140)
        active = self.mapper.map_snapshot(active_snapshot, now_ms=0)
        self.assertEqual(active["mode"], "driving")

        hold_snapshot = make_snapshot(right_trigger=100, seq=2)
        hold = self.mapper.map_snapshot(hold_snapshot, now_ms=10)
        self.assertEqual(hold["mode"], "driving")

        release_snapshot = make_snapshot(right_trigger=70, seq=3)
        release = self.mapper.map_snapshot(release_snapshot, now_ms=20)
        self.assertEqual(release["mode"], "armed_neutral")

    def test_turn_deadzone_zeroes_small_noise(self):
        snapshot = make_snapshot(right_trigger=300, left_stick_x=180)
        mapped = self.mapper.map_snapshot(snapshot, now_ms=0)

        self.assertEqual(mapped["mode"], "driving")
        self.assertEqual(mapped["turn_percent"], 0)

    def test_turn_maps_positive_outside_deadzone(self):
        snapshot = make_snapshot(right_trigger=300, left_stick_x=700)
        mapped = self.mapper.map_snapshot(snapshot, now_ms=0)

        self.assertEqual(mapped["mode"], "driving")
        self.assertGreater(mapped["turn_percent"], 0)

    def test_zero_throttle_with_turn_stays_stopped(self):
        snapshot = make_snapshot(left_stick_x=800)
        mapped = self.mapper.map_snapshot(snapshot, now_ms=0)

        self.assertEqual(mapped["mode"], "armed_neutral")
        self.assertEqual(mapped["throttle_percent"], 0)
        self.assertGreater(mapped["turn_percent"], 0)

    def test_stale_snapshot_goes_inactive(self):
        snapshot = make_snapshot(right_trigger=500, received_at_ms=0)
        mapped = self.mapper.map_snapshot(snapshot, now_ms=300)

        self.assertEqual(mapped["mode"], "inactive")
        self.assertEqual(mapped["reason"], "snapshot_stale")

    def test_disconnected_snapshot_goes_inactive(self):
        snapshot = make_snapshot(connected=False, received_at_ms=0)
        mapped = self.mapper.map_snapshot(snapshot, now_ms=0)

        self.assertEqual(mapped["mode"], "inactive")
        self.assertEqual(mapped["reason"], "controller_disconnected")

    def test_repeated_snapshot_seq_still_maps_to_driving(self):
        first = make_snapshot(right_trigger=400, seq=12)
        second = make_snapshot(right_trigger=400, seq=12, received_at_ms=50)

        mapped_first = self.mapper.map_snapshot(first, now_ms=0)
        mapped_second = self.mapper.map_snapshot(second, now_ms=50)

        self.assertEqual(mapped_first["mode"], "driving")
        self.assertEqual(mapped_second["mode"], "driving")


if __name__ == "__main__":
    unittest.main()
