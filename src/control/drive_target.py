class DriveTarget:
    """Snapshot of the accepted robot-level drive target.

    This value object carries the current requested drivetrain motion as seen by
    the closed-loop controller. It is intentionally robot-level state, not the
    final per-track PID setpoints derived later inside the drivetrain
    controller.
    """

    def __init__(self, target_rpm, target_angle, target_direction, last_seq, command_age_ms):
        self.target_rpm = target_rpm
        self.target_angle = target_angle
        self.target_direction = target_direction
        self.last_seq = last_seq
        self.command_age_ms = command_age_ms

    def get_signed_target_rpm(self):
        if self.target_direction == 0:
            return self.target_rpm
        return -self.target_rpm
