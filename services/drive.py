def sync_tracks_straight(
    target_rpm: float,
    measured_rpm_left: float,
    measured_rpm_right: float,
    prev_pwm_left: int,
    prev_pwm_right: int,
    kp: float = 0.5
) -> tuple[int, int]:
    """
    Sync both tracks to maintain straight driving at the target RPM.
    Only reduces PWM on the faster track.
    Returns: (new_pwm_left, new_pwm_right)
    """
    # Calculate RPM errors
    error_left = target_rpm - measured_rpm_left
    error_right = target_rpm - measured_rpm_right

    # Determine which track is faster, and only correct the faster one
    if measured_rpm_left > measured_rpm_right:
        correction = kp * (measured_rpm_left - measured_rpm_right)
        pwm_left = max(0, prev_pwm_left - int(correction))
        pwm_right = prev_pwm_right
    elif measured_rpm_right > measured_rpm_left:
        correction = kp * (measured_rpm_right - measured_rpm_left)
        pwm_right = max(0, prev_pwm_right - int(correction))
        pwm_left = prev_pwm_left
    else:
        pwm_left = prev_pwm_left
        pwm_right = prev_pwm_right

    return pwm_left, pwm_right
