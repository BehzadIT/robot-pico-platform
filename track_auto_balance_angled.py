import utime
from machine import Pin, PWM
from services.dual_rpm_pio_test import PIOQuadratureCounter, read_rpms


# def scale_factor_from_angle(angle_deg):
#     """
#     Returns speed scaling factor for inner track given turn angle (degrees).
#     +90° = full right pivot, 0 = straight, -90° = full left pivot.
#     Linear scaling: 1.0 (straight), 0.0 (pivot).
#     """
#     angle = max(min(angle_deg, 90), -90)
#     return 1.0 - abs(angle) / 90.0
def scale_inner_pwm(inner_pwm, angle):
    scale = 1 - abs(angle) / 90.0
    return inner_pwm * scale


def normalize_inner_rpm(inner_rpm, angle):
    scale = 1 - abs(angle) / 90.0
    return inner_rpm / scale


def sync_tracks_pid_angle(
        measured_rpm_left,
        measured_rpm_right,
        prev_pwm_left,
        prev_pwm_right,
        prev_error,
        integral,
        dt,
        target_rpm,
        angle_deg,
        kp=7.0,
        ki=1.2,
        kd=0.5,
        min_pwm=0,
        max_pwm=65535,
        integral_min=-50.0,
        integral_max=50.0,
        max_rpm=330
):
    print("measured_rpm_left:", measured_rpm_left, "measured_rpm_right:", measured_rpm_right)
    if angle_deg > 0:
        outer_track = 'left'
        inner_track = 'right'
        target_left_rpm = target_rpm
        # target_right_rpm = calculate_inner_rpm(target_rpm, angle_deg)
        normalized_measured_left = normalize_inner_rpm(measured_rpm_left, angle_deg)
        normalized_measured_right = measured_rpm_right

    else:
        outer_track = 'right'
        inner_track = 'left'
        target_right_rpm = target_rpm
        # target_left_rpm = calculate_inner_rpm(target_rpm, angle_deg)
        normalized_measured_left = normalize_inner_rpm(measured_rpm_left, angle_deg)
        normalized_measured_right = measured_rpm_right

    # min_scale = 0.05
    # safe_scale = max(scale, min_scale)
    # target_rpm_outer = target_rpm
    # target_rpm_inner = calculate_inner_rpm(target_rpm, angle_deg)

    # base_pwm_outer = int((abs(target_rpm_outer) / max_rpm) * max_pwm)
    # base_pwm_inner = int((abs(target_rpm_inner) / max_rpm) * max_pwm)

    integral_locked = None  # Track if anti-windup is active

    if normalized_measured_left < normalized_measured_right:
        error = normalized_measured_left - normalized_measured_right
        integral += error * dt
        # Anti-windup clamp with lock
        if integral > integral_max:
            integral = integral_max
            integral_locked = 'max'
        elif integral < integral_min:
            integral = integral_min
            integral_locked = 'min'
        derivative = (error - prev_error) / dt if dt > 0 else 0
        correction = int(kp * abs(error) + ki * abs(integral) + kd * abs(derivative))
        pwm_left = max(min_pwm, min(prev_pwm_left - correction, max_pwm))
        pwm_right = prev_pwm_right
        prev_error = error
        correction_side = "left"
    elif normalized_measured_right < normalized_measured_left:
        error = normalized_measured_right - normalized_measured_left
        integral += error * dt
        # Anti-windup clamp with lock
        if integral > integral_max:
            integral = integral_max
            integral_locked = 'max'
        elif integral < integral_min:
            integral = integral_min
            integral_locked = 'min'
        derivative = (error - prev_error) / dt if dt > 0 else 0
        correction = int(kp * abs(error) + ki * abs(integral) + kd * abs(derivative))
        pwm_right = max(min_pwm, min(prev_pwm_right - correction, max_pwm))
        pwm_left = prev_pwm_left
        prev_error = error
        correction_side = "right"
    else:
        error = 0
        correction = 0
        derivative = 0
        pwm_left = prev_pwm_left
        pwm_right = prev_pwm_right
        correction_side = "synced"

    if angle_deg > 0:
        pwm_right = scale_inner_pwm(pwm_right, angle_deg)
    else:
        pwm_left = scale_inner_pwm(pwm_left, angle_deg)

    # Collaborative PID: compare measured_outer to normalized inner

    # print("Measured inner RPM:", measured_inner)
    # print("Measured outer RPM:", measured_outer)
    # print("Normalized inner RPM:", normalized_inner)
    #
    # error = measured_outer - normalized_inner
    #
    # integral += error * dt
    # integral_locked = None
    # if integral > integral_max:
    #     integral = integral_max
    #     integral_locked = 'max'
    # elif integral < integral_min:
    #     integral = integral_min
    #     integral_locked = 'min'
    # derivative = (error - prev_error) / dt if dt > 0 else 0
    # correction = int(kp * error + ki * integral + kd * derivative)
    #
    # # Collaborative update: adjust BOTH tracks
    # pwm_outer = max(min_pwm, min(base_pwm_outer - correction, max_pwm))
    # pwm_inner = max(min_pwm, min(base_pwm_inner + correction, max_pwm))
    #
    # if angle_deg > 0:
    #     pwm_left = pwm_outer
    #     pwm_right = pwm_inner
    # else:
    #     pwm_right = pwm_outer
    #     pwm_left = pwm_inner
    return pwm_left, pwm_right, prev_error, integral, error, correction, derivative, correction_side, integral_locked, outer_track, inner_track


# ======================= Example Main Loop ==========================
if __name__ == "__main__":
    # Motor driver pins
    PWM1 = PWM(Pin(8))  # Right
    DIR1 = Pin(9, Pin.OUT)
    PWM2 = PWM(Pin(6))  # Left
    DIR2 = Pin(7, Pin.OUT)
    PWM1.freq(1000)
    PWM2.freq(1000)

    # Encoder pins
    enc_right = PIOQuadratureCounter(0, 10, 11)
    enc_left = PIOQuadratureCounter(1, 20, 21)

    # Start both tracks forward
    DIR1.value(1)
    DIR2.value(1)

    # User commands
    target_rpm = 150.0  # Example: outer track RPM
    angle_deg = 70.0  # Example: sharp right turn (+ is right, - is left)

    # Initial PID state
    prev_error = 0.0
    integral = 0.0

    # Initial PWM based on target and angle
    # scale = scale_factor_from_angle(angle_deg)
    MAX_RPM = 330  # Adjust for your real no-load RPM
    # base_pwm_outer = int((target_rpm / MAX_RPM) * 65535)
    # base_pwm_inner = int((target_rpm * scale / MAX_RPM) * 65535)
    # if angle_deg > 0:
    #     pwm_left = base_pwm_outer
    #     pwm_right = base_pwm_inner
    # else:
    #     pwm_right = base_pwm_outer
    #     pwm_left = base_pwm_inner

    NUM_STEPS = 100
    UPDATE_INTERVAL = 0.05  # 50ms
    pwm_left = 0
    pwm_right = 0

    for step in range(NUM_STEPS):
        measured_rpm_right, measured_rpm_left = read_rpms(
            enc_right, enc_left, interval_sec=UPDATE_INTERVAL
        )
        (pwm_left, pwm_right, prev_error, integral, error, correction, derivative, correction_side, integral_locked,
         outer_track, inner_track) = sync_tracks_pid_angle(
            measured_rpm_left, measured_rpm_right,
            prev_pwm_left=pwm_left, prev_pwm_right=pwm_right,
            prev_error=prev_error, integral=integral,
            dt=UPDATE_INTERVAL,
            target_rpm=target_rpm,
            angle_deg=angle_deg,
            kp=7.0, ki=1.2, kd=0.5,
            integral_min=-50.0, integral_max=50.0,
            max_rpm=MAX_RPM
        )

        # prev_error = error

        # Write to PWM
        PWM1.duty_u16(int(pwm_right))
        PWM2.duty_u16(int(pwm_left))
        lock_str = f" | Integral lock: {integral_locked.upper()}" if integral_locked else ""
        print(
            f"Step {step:3d} | RPM L/R: {measured_rpm_left:7.2f}/{measured_rpm_right:7.2f} | "
            f"Angle: {angle_deg:6.1f} | "
            f"Error: {error:7.2f} | Correction: {correction:6d} | Deriv: {derivative:7.2f} | "
            f"Integral: {integral:8.2f} | PWM L/R: {pwm_left:7.2f}/{pwm_right:7.2f} | "
            f"Outer: {outer_track} | Inner: {inner_track}{lock_str}"
        )
        utime.sleep(UPDATE_INTERVAL)

    print("Done. Stopping motors.")
    PWM1.duty_u16(0)
    PWM2.duty_u16(0)
    enc_right.deinit()
    enc_left.deinit()
