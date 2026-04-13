WIFI_CONFIG = {
    "connect_timeout_s": 20,
    "max_attempts": 3,
    "retry_backoff_ms": 1000,
    "pre_reset_delay_ms": 300,
    "post_activate_delay_ms": 300,
    "status_poll_ms": 250,
}

DRIVETRAIN_CONFIG = {
    # Disabled by default while stabilizing the MicroPython control loop.
    # Re-enable after the underlying stop/restart fault is understood.
    "enable_watchdog": True,
    # For later rollout, require one clean stop cycle before arming the
    # watchdog. That keeps the first watchdog-enabled test away from cold-start
    # startup paths and focuses it on already-stable teleop behavior.
    "watchdog_arm_after_first_stop": True,
    # When re-enabled, keep the timeout comfortably above normal stop/start
    # and logging bursts so the watchdog only catches real wedges.
    "watchdog_timeout_ms": 4000,
}

CONTROLLER_UART_CONFIG = {
    "enabled": True,
    "uart_id": 1,
    "baud": 115200,
    "tx_pin": 4,
    "rx_pin": 5,
    "read_chunk": 64,
    "poll_ms": 10,
    "diag_sample_ms": 500,
}

CONTROLLER_DRIVE_CONFIG = {
    "enabled": True,
    "publish_period_ms": 50,
    "snapshot_stale_ms": 250,
    # Microsoft XInput documents 30/255 as the trigger threshold for treating
    # a trigger as intentionally pressed. The ESP32 transport normalizes
    # triggers to 0..1000, so 30/255 scales to about 118; we round that to 120
    # for a readable, conservative robot-driving threshold.
    "trigger_press_threshold": 120,
    # Use a lower release threshold to add hysteresis. That keeps the robot
    # from chattering between stop and low-speed drive when the operator eases
    # off a trigger or when the trigger rests near its neutral edge.
    "trigger_release_threshold": 80,
    "turn_deadzone": 240,
}
