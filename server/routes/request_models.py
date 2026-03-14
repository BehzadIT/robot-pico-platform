from services.drivetrain_constants import SteeringLimits


class ApiDriveRequest:
    """Parsed transport-level drive request.

    The Pico accepts both short WebSocket field names and the longer HTTP field
    names. Missing angle means "drive straight" for backward compatibility with
    older clients.
    """

    def __init__(self, data):
        self.target_rpm = int(data.get('r', data.get('target_rpm', 0)))
        self.target_direction = int(data.get('d', data.get('target_direction', 1)))
        raw_angle = data.get('a', data.get('target_angle', SteeringLimits.ANGLE_DEFAULT))
        self.target_angle = self._clamp_angle(int(raw_angle))
        self.seq = self._to_optional_int(data.get('s', data.get('seq')))
        self.sent_at_ms = self._to_optional_int(data.get('m', data.get('sent_at_ms')))

    @staticmethod
    def _to_optional_int(value):
        if value is None:
            return None
        return int(value)

    @staticmethod
    def _clamp_angle(value):
        return max(SteeringLimits.ANGLE_MIN, min(SteeringLimits.ANGLE_MAX, value))
