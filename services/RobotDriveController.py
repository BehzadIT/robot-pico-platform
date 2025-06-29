import utime

class RobotDriveController:
    def __init__(self):
        self._target_rpm = 0
        # Min Angle: -90 Maximum left turn (left track nearly stopped, right track max)
        # Max Angle: +90 Maximum right turn (right track nearly stopped, left track max)
        self._target_angle = 0
        # Direction: 0 for forward, 1 for reverse
        self._direction = 0
        self._last_command_time = utime.ticks_ms()
        # Add other PID state vars here if you wish

    # Exposed public API
    def drive(self, rpm, direction=1):
        self._target_rpm = rpm
        self._direction = direction
        self._last_command_time = utime.ticks_ms()

    def turn(self, angle):
        self._target_angle = angle
        self._last_command_time = utime.ticks_ms()

    def stop(self):
        self._target_rpm = 0
        self._last_command_time = utime.ticks_ms()

    # Internal: called by your main control loop
    def get_targets(self):
        return self._target_rpm, self._target_angle, self._direction

    def is_timeout(self, timeout_ms=2000):
        return utime.ticks_diff(utime.ticks_ms(), self._last_command_time) > timeout_ms

# this is the singleton instance of the driver controller
driverController = RobotDriveController()