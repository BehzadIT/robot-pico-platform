import utime
import _thread
from log import *
from server.routes.request_models import ApiDriveRequest
from services.robot_cross_coupled_pid import robot_pid

nav_lock = _thread.allocate_lock()


class RobotNavigationController:
    def __init__(self):
        self._target_rpm = 0
        # Min Angle: -90 Maximum left turn (left track nearly stopped, right track max)
        # Max Angle: +90 Maximum right turn (right track nearly stopped, left track max)
        self._target_angle = 0
        # Direction: 0 for forward, 1 for reverse
        self._target_direction = 0
        self._last_command_time = utime.ticks_ms()
        # Add other PID state vars here if you wish

    # Exposed public API
    def drive(self, drive_request: ApiDriveRequest):
        with nav_lock:
            robot_pid.start(driverController)
            self._target_rpm = drive_request.target_rpm
            self._target_direction = drive_request.target_direction
            self._last_command_time = utime.ticks_ms()

    def turn(self, angle):
        with nav_lock:
            self._target_angle = angle
            self._last_command_time = utime.ticks_ms()

    def stop(self):
        with nav_lock:
            robot_pid.terminate_thread()
            logd("Stop command received")
            self._target_rpm = 0
            self._last_command_time = utime.ticks_ms()

    # Internal: called by your main control loop
    def get_navigation_params(self):
        with nav_lock:
            return NavigationParams(
                self._target_rpm,
                self._target_angle,
                self._target_direction)

    def is_timeout(self, timeout_ms=2000):
        with nav_lock:
            return utime.ticks_diff(utime.ticks_ms(), self._last_command_time) > timeout_ms


# this is the singleton instance of the driver controller
driverController = RobotNavigationController()


class NavigationParams:
    def __init__(self, rpm, angle, direction):
        self.target_rpm = rpm
        self.target_angle = angle
        self.target_direction = direction

    def get_signed_target_rpm(self):
        if self.target_direction == 0:
            return self.target_rpm
        else:
            return -self.target_rpm
