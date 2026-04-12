import utime

from src.control.drive_target import DriveTarget
from src.control.drivetrain_constants import SteeringLimits


class DriveTargetStore:
    """Mutable store for the latest accepted drivetrain target state.

    This class owns the robot-level target values and freshness timestamp that
    the drivetrain controller reads on each control tick. It does not decide
    which command source is allowed to update the target; that policy remains in
    `DriveCommandManager`.
    """

    def __init__(self):
        self._target_rpm = 0
        self._target_angle = SteeringLimits.ANGLE_DEFAULT
        self._target_direction = 0
        self._last_command_time = utime.ticks_ms()
        self._last_seq = -1

    def update(self, drive_request, now_ms=None):
        if now_ms is None:
            now_ms = utime.ticks_ms()
        self._target_rpm = drive_request.target_rpm
        self._target_direction = drive_request.target_direction
        self._target_angle = self._clamp_angle(drive_request.target_angle)
        if drive_request.seq is not None:
            self._last_seq = drive_request.seq
        self._last_command_time = now_ms

    def set_turn_angle(self, angle, now_ms=None):
        if now_ms is None:
            now_ms = utime.ticks_ms()
        self._target_angle = self._clamp_angle(angle)
        self._last_command_time = now_ms

    def reset(self, now_ms=None):
        if now_ms is None:
            now_ms = utime.ticks_ms()
        self._target_rpm = 0
        self._target_angle = SteeringLimits.ANGLE_DEFAULT
        self._last_command_time = now_ms

    def snapshot(self):
        return DriveTarget(
            self._target_rpm,
            self._target_angle,
            self._target_direction,
            self._last_seq,
            self.command_age_ms(),
        )

    def command_age_ms(self):
        return utime.ticks_diff(utime.ticks_ms(), self._last_command_time)

    @property
    def target_rpm(self):
        return self._target_rpm

    @property
    def last_seq(self):
        return self._last_seq

    @staticmethod
    def _clamp_angle(angle):
        return max(SteeringLimits.ANGLE_MIN, min(SteeringLimits.ANGLE_MAX, int(angle)))
