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
