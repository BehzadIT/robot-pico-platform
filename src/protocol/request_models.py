from src.control.drivetrain_constants import MotorLimits, SteeringLimits


class ApiDriveRequest:
    """Parsed transport-level drive request.

    WebSocket now carries signed operator intent as throttle/turn percentages,
    while the HTTP route retains the older RPM/direction/angle shape for
    manual or legacy use.
    """

    def __init__(self, data):
        self.seq = self._to_optional_int(data.get('q', data.get('seq')))
        self.sent_at_ms = self._to_optional_int(data.get('w', data.get('sent_at_ms')))

        if 'k' in data:
            self.throttle_percent = self._clamp_percent(int(data.get('t', 0)))
            self.turn_percent = self._clamp_percent(int(data.get('r', 0)))
            signed_rpm = self._percent_to_signed_rpm(self.throttle_percent)
            self.target_rpm = self._normalize_rpm(abs(signed_rpm))
            self.target_direction = 0 if signed_rpm >= 0 else 1
            self.target_angle = self._percent_to_angle(self.turn_percent)
            return

        raw_rpm = int(data.get('target_rpm', 0))
        self.target_rpm = self._normalize_rpm(raw_rpm)
        self.target_direction = int(data.get('target_direction', 1))
        raw_angle = data.get('target_angle', SteeringLimits.ANGLE_DEFAULT)
        self.target_angle = self._clamp_angle(int(raw_angle))
        self.throttle_percent = self._rpm_to_percent(self.target_rpm, self.target_direction)
        self.turn_percent = self._angle_to_percent(self.target_angle)

    @classmethod
    def from_intent(cls, throttle_percent, turn_percent, seq=None, sent_at_ms=None):
        """Build one drive request directly from robot-level operator intent.

        This helper is used by non-HTTP, non-WebSocket control paths such as
        the Pico-side controller adapter. It preserves one normalization path
        from signed `throttle/turn` intent into the drivetrain's internal
        `rpm/direction/angle` target representation.
        """
        # MicroPython does not expose `cls.__new__` the same way CPython does
        # for bypassing `__init__`, so use `object.__new__` directly here.
        instance = object.__new__(cls)
        instance.seq = cls._to_optional_int(seq)
        instance.sent_at_ms = cls._to_optional_int(sent_at_ms)
        instance.throttle_percent = cls._clamp_percent(int(throttle_percent))
        instance.turn_percent = cls._clamp_percent(int(turn_percent))
        signed_rpm = cls._percent_to_signed_rpm(instance.throttle_percent)
        instance.target_rpm = cls._normalize_rpm(abs(signed_rpm))
        instance.target_direction = 0 if signed_rpm >= 0 else 1
        instance.target_angle = cls._percent_to_angle(instance.turn_percent)
        return instance

    @staticmethod
    def _to_optional_int(value):
        if value is None:
            return None
        return int(value)

    @staticmethod
    def _clamp_angle(value):
        return max(SteeringLimits.ANGLE_MIN, min(SteeringLimits.ANGLE_MAX, value))

    @staticmethod
    def _normalize_rpm(value):
        if abs(value) < MotorLimits.MIN_EFFECTIVE_RPM:
            return 0
        return value

    @staticmethod
    def _clamp_percent(value):
        return max(-100, min(100, value))

    @staticmethod
    def _percent_to_signed_rpm(value):
        return int(round((value / 100.0) * MotorLimits.RPM_MAX))

    @staticmethod
    def _percent_to_angle(value):
        return ApiDriveRequest._clamp_angle(int(round((value / 100.0) * SteeringLimits.ANGLE_MAX)))

    @staticmethod
    def _rpm_to_percent(target_rpm, target_direction):
        if target_rpm == 0:
            return 0
        sign = 1 if target_direction == 0 else -1
        return ApiDriveRequest._clamp_percent(int(round((target_rpm / float(MotorLimits.RPM_MAX)) * 100)) * sign)

    @staticmethod
    def _angle_to_percent(target_angle):
        if target_angle == 0:
            return 0
        return ApiDriveRequest._clamp_percent(int(round((target_angle / float(SteeringLimits.ANGLE_MAX)) * 100)))
