import utime
import _thread
from src.protocol.request_models import ApiDriveRequest
from src.support.logging import *
from src.control.drivetrain_constants import SafetyTiming, SteeringLimits

nav_lock = _thread.allocate_lock()


class RobotNavigationController:
    def __init__(self):
        self._target_rpm = 0
        # Steering convention shared with transport/UI/control:
        # - negative angle = steer left
        # - positive angle = steer right
        # With inner-track slowdown, the inside track is reduced toward zero as
        # the command approaches +/-90 degrees.
        self._target_angle = SteeringLimits.ANGLE_DEFAULT
        # Direction: 0 for forward, 1 for reverse
        self._target_direction = 0
        self._last_command_time = utime.ticks_ms()
        self._active_controller_id = None
        self._last_seq = -1
        self._timeout_fence_seq = -1
        self._timeout_guard_until_ms = utime.ticks_ms()

    # Exposed public API
    def drive(self, drive_request: ApiDriveRequest, controller_id=None):
        result = self.accept_drive_request(drive_request, controller_id=controller_id)
        return result["ok"]

    def accept_drive_request(self, drive_request: ApiDriveRequest, controller_id=None):
        with nav_lock:
            if controller_id is not None and self._active_controller_id not in (None, controller_id):
                return {"ok": False, "code": "ic", "detail": "controller does not own drivetrain"}
            if drive_request.seq is not None and drive_request.seq <= max(self._last_seq, self._timeout_fence_seq):
                return {"ok": False, "code": "st", "detail": "stale sequence"}

            now_ms = utime.ticks_ms()
            if utime.ticks_diff(self._timeout_guard_until_ms, now_ms) > 0:
                return {"ok": False, "code": "si", "detail": "timeout stop settling"}

            self._target_rpm = drive_request.target_rpm
            self._target_direction = drive_request.target_direction
            self._target_angle = self._clamp_angle(drive_request.target_angle)
            self._active_controller_id = controller_id
            if drive_request.seq is not None:
                self._last_seq = drive_request.seq
            self._last_command_time = now_ms
            return {"ok": True}

    def revoke_drive_request(self, seq=None, controller_id=None, reason="drive_rejected"):
        with nav_lock:
            if controller_id is not None and self._active_controller_id not in (None, controller_id):
                return False
            if seq is not None and self._last_seq != seq:
                return False
            self._target_rpm = 0
            self._target_angle = SteeringLimits.ANGLE_DEFAULT
            self._active_controller_id = None
            self._last_command_time = utime.ticks_ms()
            logw("Drive request revoked: %s" % reason)
            return True

    def turn(self, angle):
        with nav_lock:
            self._target_angle = self._clamp_angle(angle)
            self._last_command_time = utime.ticks_ms()

    def stop(self, controller_id=None, reason="manual_stop"):
        with nav_lock:
            if controller_id is not None and self._active_controller_id not in (None, controller_id):
                logw("Ignoring stop from non-active controller")
                return False
            self._target_rpm = 0
            self._target_angle = SteeringLimits.ANGLE_DEFAULT
            self._active_controller_id = None
            self._last_command_time = utime.ticks_ms()
            logi("Controller stop requested: %s" % reason)
            return True

    # Internal: called by your main control loop
    def get_navigation_params(self):
        with nav_lock:
            return NavigationParams(
                self._target_rpm,
                self._target_angle,
                self._target_direction,
                self._last_seq,
                utime.ticks_diff(utime.ticks_ms(), self._last_command_time),
            )

    def is_timeout(self, timeout_ms=SafetyTiming.COMMAND_TIMEOUT_MS):
        with nav_lock:
            if self._target_rpm == 0:
                return False
            return utime.ticks_diff(utime.ticks_ms(), self._last_command_time) > timeout_ms

    def command_age_ms(self):
        with nav_lock:
            return utime.ticks_diff(utime.ticks_ms(), self._last_command_time)

    def is_fresh_sequence(self, seq, controller_id=None):
        with nav_lock:
            if seq is None:
                return True
            if controller_id is not None and self._active_controller_id not in (None, controller_id):
                return False
            return seq > max(self._last_seq, self._timeout_fence_seq)

    def clear_controller_if_active(self, controller_id, reason="disconnect_stop"):
        with nav_lock:
            if self._active_controller_id != controller_id:
                return False
            self._target_rpm = 0
            self._target_angle = SteeringLimits.ANGLE_DEFAULT
            self._active_controller_id = None
            self._last_command_time = utime.ticks_ms()
            logw("Active controller lost: %s" % reason)
            return True

    def timeout_stop(self, timeout_ms=SafetyTiming.COMMAND_TIMEOUT_MS):
        with nav_lock:
            if self._target_rpm == 0:
                return None
            age_ms = utime.ticks_diff(utime.ticks_ms(), self._last_command_time)
            if age_ms <= timeout_ms:
                return None
            snapshot = {
                "age_ms": age_ms,
                "last_seq": self._last_seq,
                "controller_id": self._active_controller_id,
            }
            self._target_rpm = 0
            self._target_angle = SteeringLimits.ANGLE_DEFAULT
            if self._last_seq is not None:
                self._timeout_fence_seq = max(self._timeout_fence_seq, self._last_seq)
            self._active_controller_id = None
            now_ms = utime.ticks_ms()
            self._last_command_time = now_ms
            self._timeout_guard_until_ms = utime.ticks_add(now_ms, SafetyTiming.TIMEOUT_RESTART_GUARD_MS)
            logw("Command timeout reached; stopping controller")
            return snapshot

    @staticmethod
    def _clamp_angle(angle):
        return max(SteeringLimits.ANGLE_MIN, min(SteeringLimits.ANGLE_MAX, int(angle)))


# this is the singleton instance of the driver controller
driverController = RobotNavigationController()


class NavigationParams:
    def __init__(self, rpm, angle, direction, last_seq, command_age_ms):
        self.target_rpm = rpm
        self.target_angle = angle
        self.target_direction = direction
        self.last_seq = last_seq
        self.command_age_ms = command_age_ms

    def get_signed_target_rpm(self):
        if self.target_direction == 0:
            return self.target_rpm
        else:
            return -self.target_rpm
