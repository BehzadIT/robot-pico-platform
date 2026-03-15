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
    "enable_watchdog": False,
}
