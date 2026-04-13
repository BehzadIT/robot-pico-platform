import utime


class ControllerDriveMapper:
    """Map transport-oriented controller snapshots into robot drive intent.

    This adapter keeps controller semantics on the Pico. It consumes normalized
    ESP32 snapshots, applies robot-specific dead zones and safety rules, then
    returns the robot-level intent to publish through the existing drive path.
    """

    def __init__(self, config):
        self._config = config
        self._throttle_active = False

    def reset(self):
        """Return the mapper to a neutral hysteresis state."""
        self._throttle_active = False

    def map_snapshot(self, snapshot, now_ms=None):
        """Map one transport snapshot into a controller-drive mode/result.

        Returned modes are:
        - `driving`: valid snapshot plus active signed throttle
        - `armed_neutral`: valid snapshot but neutral throttle
        - `inactive`: no usable controller input because the snapshot is
          missing, stale, or disconnected
        """
        if now_ms is None:
            now_ms = utime.ticks_ms()

        stale_ms = self._config["snapshot_stale_ms"]
        if snapshot is None:
            self.reset()
            return {"mode": "inactive", "reason": "no_snapshot"}

        age_ms = utime.ticks_diff(now_ms, snapshot.received_at_ms)
        if age_ms > stale_ms:
            self.reset()
            return {"mode": "inactive", "reason": "snapshot_stale", "age_ms": age_ms}

        if not snapshot.connected:
            self.reset()
            return {"mode": "inactive", "reason": "controller_disconnected", "age_ms": age_ms}

        throttle_percent = self._map_signed_throttle(
            snapshot.left_trigger, snapshot.right_trigger
        )
        turn_percent = self._map_turn(snapshot.left_stick_x)

        if throttle_percent == 0:
            return {
                "mode": "armed_neutral",
                "reason": "neutral_throttle",
                "age_ms": age_ms,
                "throttle_percent": 0,
                "turn_percent": turn_percent,
                "snapshot_seq": snapshot.seq,
            }

        return {
            "mode": "driving",
            "age_ms": age_ms,
            "throttle_percent": throttle_percent,
            "turn_percent": turn_percent,
            "snapshot_seq": snapshot.seq,
        }

    def _map_signed_throttle(self, left_trigger, right_trigger):
        """Convert trigger difference into signed throttle with hysteresis."""
        signed_raw = int(right_trigger) - int(left_trigger)
        magnitude = abs(signed_raw)
        sign = 1 if signed_raw >= 0 else -1
        press_threshold = int(self._config["trigger_press_threshold"])
        release_threshold = int(self._config["trigger_release_threshold"])

        if self._throttle_active:
            if magnitude < release_threshold:
                self._throttle_active = False
                return 0
            threshold = release_threshold
        else:
            if magnitude < press_threshold:
                return 0
            self._throttle_active = True
            threshold = press_threshold

        scaled = self._scale_with_threshold(
            magnitude,
            threshold,
            1000,
        )
        return max(-100, min(100, sign * scaled))

    def _map_turn(self, axis_x):
        return self._scale_signed_with_deadzone(axis_x, int(self._config["turn_deadzone"]), 1000)

    @staticmethod
    def _scale_with_threshold(value, threshold, maximum):
        """Scale one magnitude after removing its neutral threshold band."""
        if value <= threshold:
            return 0
        usable_range = maximum - threshold
        if usable_range <= 0:
            return 100
        scaled = int(round(((value - threshold) * 100.0) / usable_range))
        if scaled < 0:
            return 0
        if scaled > 100:
            return 100
        return scaled

    @classmethod
    def _scale_signed_with_deadzone(cls, value, deadzone, maximum):
        """Scale one signed axis after applying its center dead zone."""
        sign = 1 if value >= 0 else -1
        magnitude = abs(int(value))
        scaled = cls._scale_with_threshold(magnitude, deadzone, maximum)
        return sign * scaled
