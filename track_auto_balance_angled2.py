# track_auto_balance2.py
import utime
from machine import Pin, PWM
from services.dual_rpm_pio_test import PIOQuadratureCounter, read_rpms

def pid_control(
    target_rpm,
    measured_rpm,
    prev_error,
    integral,
    dt,
    kp,
    ki,
    kd,
    min_pwm,
    max_pwm,
    min_integral,
    max_integral,
    min_start_pwm,
    label="",
):
    # PID error terms
    error = target_rpm - measured_rpm
    integral += error * dt

    integral_locked = None
    if integral > max_integral:
        integral = max_integral
        integral_locked = "MAX"
    elif integral < min_integral:
        integral = min_integral
        integral_locked = "MIN"

    derivative = (error - prev_error) / dt if dt > 0 else 0

    # PID formula
    output = kp * error + ki * integral + kd * derivative
    pwm = int(max(min_pwm, min(output, max_pwm)))

    # Apply minimum PWM threshold for movement
    from log import logd
    if abs(target_rpm) > 1.0 and abs(pwm) < min_start_pwm:
        pwm = min_start_pwm if pwm >= 0 else -min_start_pwm

    logd(
        f"[{label}] Target: {target_rpm:.2f}, Measured: {measured_rpm:.2f}, Error: {error:.2f}, "
        f"Int: {integral:.2f} ({integral_locked}), Deriv: {derivative:.2f}, "
        f"PID: {output:.0f}, PWM: {pwm}"
    )

    return pwm, error, integral, derivative, integral_locked

if __name__ == "__main__":
    # Motor driver pins (adjust if needed)
    PWM1 = PWM(Pin(8))  # Right
    DIR1 = Pin(9, Pin.OUT)
    PWM2 = PWM(Pin(6))  # Left
    DIR2 = Pin(7, Pin.OUT)
    PWM1.freq(1000)
    PWM2.freq(1000)

    # Encoder pins for Pololu 37D
    enc_right = PIOQuadratureCounter(0, 10, 11)
    enc_left  = PIOQuadratureCounter(1, 20, 21)

    # Start both tracks forward
    DIR1.value(0)
    DIR2.value(0)

    # --- Parameters ---
    NUM_STEPS = 100
    BASE_RPM = 150   # Outer track target (try 100–300 to see behavior)
    ANGLE = 70.0     # Turning angle (-90...0...+90, positive=right)
    UPDATE_INTERVAL = 0.05  # seconds

    # PID gains
    KP = 7.0
    KI = 1.2
    KD = 0.5
    MIN_PWM = 0
    MAX_PWM = 65535
    MIN_INTEGRAL = -50.0
    MAX_INTEGRAL = 50.0

    # Minimum PWM to move the motor (experimentally determined)
    MIN_START_PWM = int(0.20 * MAX_PWM)  # 20% of max (~6553)

    # Track state
    pwm_left = 0
    pwm_right = 0
    integral_left = 0
    integral_right = 0
    prev_error_left = 0
    prev_error_right = 0

    for step in range(NUM_STEPS):
        measured_rpm_right, measured_rpm_left = read_rpms(
            enc_right, enc_left, interval_sec=UPDATE_INTERVAL
        )

        # Determine outer/inner tracks (left is outer if angle > 0)
        if ANGLE >= 0:
            # Turning right: left is outer, right is inner
            target_rpm_outer = BASE_RPM
            target_rpm_inner = BASE_RPM * (1 - abs(ANGLE) / 90)
            measured_rpm_outer = measured_rpm_left
            measured_rpm_inner = measured_rpm_right
            label_outer = "OUTER-left"
            label_inner = "INNER-right"
        else:
            # Turning left: right is outer, left is inner
            target_rpm_outer = BASE_RPM
            target_rpm_inner = BASE_RPM * (1 - abs(ANGLE) / 90)
            measured_rpm_outer = measured_rpm_right
            measured_rpm_inner = measured_rpm_left
            label_outer = "OUTER-right"
            label_inner = "INNER-left"

        # Outer PID
        pwm_outer, error_outer, integral_outer, derivative_outer, integral_locked_outer = pid_control(
            target_rpm_outer,
            measured_rpm_outer,
            prev_error_left if ANGLE >= 0 else prev_error_right,
            integral_left if ANGLE >= 0 else integral_right,
            UPDATE_INTERVAL,
            KP, KI, KD,
            MIN_PWM, MAX_PWM,
            MIN_INTEGRAL, MAX_INTEGRAL,
            MIN_START_PWM,
            label=label_outer
        )

        # Inner PID (target is scaled)
        pwm_inner, error_inner, integral_inner, derivative_inner, integral_locked_inner = pid_control(
            target_rpm_inner,
            measured_rpm_inner,
            prev_error_right if ANGLE >= 0 else prev_error_left,
            integral_right if ANGLE >= 0 else integral_left,
            UPDATE_INTERVAL,
            KP, KI, KD,
            MIN_PWM, MAX_PWM,
            MIN_INTEGRAL, MAX_INTEGRAL,
            MIN_START_PWM,
            label=label_inner
        )

        # Assign PWM values to correct track
        if ANGLE >= 0:
            pwm_left = pwm_outer
            pwm_right = pwm_inner
            prev_error_left = error_outer
            prev_error_right = error_inner
            integral_left = integral_outer
            integral_right = integral_inner
        else:
            pwm_left = pwm_inner
            pwm_right = pwm_outer
            prev_error_left = error_inner
            prev_error_right = error_outer
            integral_left = integral_inner
            integral_right = integral_outer

        # Apply PWM
        from log import logi, logd
        PWM1.duty_u16(int(pwm_right))
        PWM2.duty_u16(int(pwm_left))

        logd(
            f"Step {step:3d} | ANGLE: {ANGLE:.1f} | OUTER ({label_outer.split('-')[1]}): rpm={measured_rpm_outer:.2f} | "
            f"INNER ({label_inner.split('-')[1]}): rpm={measured_rpm_inner:.2f} | Scale: {1 - abs(ANGLE)/90:.2f}"
        )
        utime.sleep(UPDATE_INTERVAL)

    logi("Done. Stopping motors.")
    PWM1.duty_u16(0)
    PWM2.duty_u16(0)

    enc_right.deinit()
    enc_left.deinit()
