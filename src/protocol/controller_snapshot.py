class ControllerSnapshotV1:
    """Receive-side DTO for ESP32 controller snapshots.

    This stays transport-oriented in Phase 2. It intentionally does not assign
    robot meaning to the controls yet.
    """

    def __init__(
        self,
        seq,
        source_uptime_ms,
        connected,
        dpad,
        buttons,
        misc_buttons,
        left_stick_x,
        left_stick_y,
        right_stick_x,
        right_stick_y,
        left_trigger,
        right_trigger,
        received_at_ms,
    ):
        self.seq = seq
        self.source_uptime_ms = source_uptime_ms
        self.connected = connected
        self.dpad = dpad
        self.buttons = buttons
        self.misc_buttons = misc_buttons
        self.left_stick_x = left_stick_x
        self.left_stick_y = left_stick_y
        self.right_stick_x = right_stick_x
        self.right_stick_y = right_stick_y
        self.left_trigger = left_trigger
        self.right_trigger = right_trigger
        self.received_at_ms = received_at_ms

    def as_dict(self):
        return {
            "seq": self.seq,
            "source_uptime_ms": self.source_uptime_ms,
            "connected": self.connected,
            "dpad": self.dpad,
            "buttons": self.buttons,
            "misc_buttons": self.misc_buttons,
            "left_stick_x": self.left_stick_x,
            "left_stick_y": self.left_stick_y,
            "right_stick_x": self.right_stick_x,
            "right_stick_y": self.right_stick_y,
            "left_trigger": self.left_trigger,
            "right_trigger": self.right_trigger,
            "received_at_ms": self.received_at_ms,
        }
