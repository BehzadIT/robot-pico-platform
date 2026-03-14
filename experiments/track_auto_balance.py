# track_auto_balance.py

from log import *
from machine import Pin, PWM
from services.dual_rpm_pio_test import PIOQuadratureCounter, read_rpms

def auto_balance_tracks(
    base_rpm,
    angle,
    measured_rpm_left,
    measured_rpm_right,
    prev_pwm_left,
    prev_pwm_right,
    kp=0.7,
    min_pwm=0,
    max_pwm=65535
):
    """
    Balances two tracks (PWM outputs) based on target speed and angle.
    Always corrects using absolute value of RPMs (speed, not direction!).
    Returns: (new_pwm_left, new_pwm_right, correction_left, correction_right, prev_pwm_left, prev_pwm_right)
    """
    # Clamp angle to -90 to +90
    angle = max(-90, min(90, angle))
    ratio = 1.0 - abs(angle) / 90.0  # 1.0 (straight), 0.0 (pivot/in-place)

    # Calculate the intended absolute target speeds
    if angle >= 0:
        target_abs_left = abs(base_rpm)
        target_abs_right = abs(base_rpm) * ratio
    else:
        target_abs_left = abs(base_rpm) * ratio
        target_abs_right = abs(base_rpm)

    # Always work with magnitudes for balancing
    abs_rpm_left = abs(measured_rpm_left)
    abs_rpm_right = abs(measured_rpm_right)

    pwm_left, pwm_right = prev_pwm_left, prev_pwm_right
    correction_left = 0
    correction_right = 0

    # For straight: equalize absolute RPMs
    if angle == 0:
        if abs_rpm_left > abs_rpm_right:
            correction = int(kp * (abs_rpm_left - abs_rpm_right))
            pwm_left = max(min_pwm, prev_pwm_left - correction)
            correction_left = pwm_left - prev_pwm_left  # should be zero or negative
        elif abs_rpm_right > abs_rpm_left:
            correction = int(kp * (abs_rpm_right - abs_rpm_left))
            pwm_right = max(min_pwm, prev_pwm_right - correction)
            correction_right = pwm_right - prev_pwm_right
    else:
        # For turning, enforce intended ratio using magnitudes
        if angle > 0:
            intended_ratio = target_abs_right / target_abs_left if target_abs_left != 0 else 1.0
            actual_ratio = abs_rpm_right / abs_rpm_left if abs_rpm_left != 0 else 1.0
            if actual_ratio > intended_ratio:
                # Right track is too fast, slow it down
                correction = int(kp * (abs_rpm_right - abs_rpm_left * intended_ratio))
                pwm_right = max(min_pwm, prev_pwm_right - correction)
                correction_right = pwm_right - prev_pwm_right
            elif actual_ratio < intended_ratio:
                # Left track is too fast, slow it down
                correction = int(kp * (abs_rpm_left - abs_rpm_right / intended_ratio))
                pwm_left = max(min_pwm, prev_pwm_left - correction)
                correction_left = pwm_left - prev_pwm_left
        elif angle < 0:
            intended_ratio = target_abs_left / target_abs_right if target_abs_right != 0 else 1.0
            actual_ratio = abs_rpm_left / abs_rpm_right if abs_rpm_right != 0 else 1.0
            if actual_ratio > intended_ratio:
                # Left track is too fast, slow it down
                correction = int(kp * (abs_rpm_left - abs_rpm_right * intended_ratio))
                pwm_left = max(min_pwm, prev_pwm_left - correction)
                correction_left = pwm_left - prev_pwm_left
            elif actual_ratio < intended_ratio:
                # Right track is too fast, slow it down
                correction = int(kp * (abs_rpm_right - abs_rpm_left / intended_ratio))
                pwm_right = max(min_pwm, prev_pwm_right - correction)
                correction_right = pwm_right - prev_pwm_right

    pwm_left = min(max(pwm_left, min_pwm), max_pwm)
    pwm_right = min(max(pwm_right, min_pwm), max_pwm)

    return pwm_left, pwm_right, correction_left, correction_right, prev_pwm_left, prev_pwm_right



# ======================= Example Main Loop ==========================
if __name__ == "__main__":
    # Example: Adapt these pins to your setup!
    # Motor driver pins (adjust if needed)
    PWM1 = PWM(Pin(8))
    DIR1 = Pin(9, Pin.OUT)
    PWM2 = PWM(Pin(6))
    DIR2 = Pin(7, Pin.OUT)
    PWM1.freq(1000)
    PWM2.freq(1000)

    # Encoder pins for Pololu 37D
    # Right: 10 (A), 11 (B)
    # Left: 20 (A), 21 (B)
    enc_right = PIOQuadratureCounter(0, 10, 11)
    enc_left  = PIOQuadratureCounter(1, 20, 21)

    # Start both tracks forward
    DIR1.value(1)
    DIR2.value(1)

    # Set initial PWM
    pwm_left = 65535
    pwm_right = 65535
    PWM1.duty_u16(pwm_right)
    PWM2.duty_u16(pwm_left)

    # Example: run for 5 seconds at max "base_rpm", angle=0 (straight)
    base_rpm = 330   # No-load max RPM (adjust if lower battery etc.)
    angle = 0        # Straight
    duration_sec = 5
    update_interval = 0.05  # 50ms = 20Hz
    steps = int(duration_sec / update_interval)
    logi("Starting auto-balance loop for {} seconds...".format(duration_sec))

    for step in range(steps):
        measured_rpm_right, measured_rpm_left = read_rpms(enc_right, enc_left, interval_sec=update_interval)
        prev_left = pwm_left
        prev_right = pwm_right

        pwm_left, pwm_right, correction_left, correction_right, _, _ = auto_balance_tracks(
            base_rpm, angle,
            measured_rpm_left, measured_rpm_right,
            pwm_left, pwm_right
        )

        PWM1.duty_u16(int(pwm_right))
        PWM2.duty_u16(int(pwm_left))

        logd(
            "Step {:3d} | PWM L: {:5d} | RPM L: {:8.2f} | PWM R: {:5d} | RPM R: {:8.2f} | "
            "Delta (L-R): {:+7.2f} | Corr L: {:+5d} | Corr R: {:+5d}".format(
                step, pwm_left, measured_rpm_left, pwm_right, measured_rpm_right,
                measured_rpm_left - measured_rpm_right,
                correction_left, correction_right
            )
        )


    logi("Done. Stopping motors.")
    PWM1.duty_u16(0)
    PWM2.duty_u16(0)

    # Clean up encoders (optional but good practice)
    enc_right.deinit()
    enc_left.deinit()
