# track_auto_balance2.py
import utime
from machine import Pin, PWM
import rp2
from dual_rpm_pio_test import PIOQuadratureCounter, read_rpms

def sync_tracks_pid(
    measured_rpm_left,
    measured_rpm_right,
    prev_pwm_left,
    prev_pwm_right,
    prev_error,
    integral,
    dt,
    kp=1.2,
    ki=0.3,
    kd=0.1,
    min_pwm=0,
    max_pwm=65535
):
    # For negative RPMs, more negative = faster in reverse
    if measured_rpm_left < measured_rpm_right:
        # LEFT is faster (more negative)
        error = measured_rpm_left - measured_rpm_right
        integral += error * dt
        derivative = (error - prev_error) / dt if dt > 0 else 0
        correction = int(kp * abs(error) + ki * abs(integral) + kd * abs(derivative))
        pwm_left = max(min_pwm, min(prev_pwm_left - correction, max_pwm))
        pwm_right = prev_pwm_right
        prev_error = error
        correction_side = "left"
    elif measured_rpm_right < measured_rpm_left:
        # RIGHT is faster (more negative)
        error = measured_rpm_right - measured_rpm_left
        integral += error * dt
        derivative = (error - prev_error) / dt if dt > 0 else 0
        correction = int(kp * abs(error) + ki * abs(integral) + kd * abs(derivative))
        pwm_right = max(min_pwm, min(prev_pwm_right - correction, max_pwm))
        pwm_left = prev_pwm_left
        prev_error = error
        correction_side = "right"
    else:
        # Perfect sync
        error = 0
        correction = 0
        derivative = 0
        pwm_left = prev_pwm_left
        pwm_right = prev_pwm_right
        correction_side = "synced"
    return pwm_left, pwm_right, prev_error, integral, error, correction, derivative, correction_side


# ======================= Example Main Loop ==========================
if __name__ == "__main__":
    # Motor driver pins (adjust if needed)
    PWM1 = PWM(Pin(8))  # Right
    DIR1 = Pin(9, Pin.OUT)
    PWM2 = PWM(Pin(6))  # Left
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

    # --- Parameters ---
    NUM_STEPS = 100  # Number of PID updates to run
    BASE_RPM = 330   # Desired base RPM (edit as needed)
    ANGLE = 0        # 0=straight, >0=right turn, <0=left turn
    UPDATE_INTERVAL = 0.05  # Loop interval in seconds

    pwm_left = 65535
    pwm_right = 65535
    prev_error = 0
    integral = 0

    for step in range(NUM_STEPS):
        measured_rpm_right, measured_rpm_left = read_rpms(
            enc_right, enc_left, interval_sec=UPDATE_INTERVAL
        )
        pwm_left, pwm_right, prev_error, integral, error, correction, derivative, correction_side = sync_tracks_pid(
            measured_rpm_left, measured_rpm_right,
            pwm_left, pwm_right,
            prev_error, integral,
            dt=UPDATE_INTERVAL,
            kp=7.0, ki=1.2, kd=0.5
        )
        PWM1.duty_u16(int(pwm_left))
        PWM2.duty_u16(int(pwm_right))
        print(
            f"Step {step:3d} | RPM L/R: {measured_rpm_left:7.2f}/{measured_rpm_right:7.2f} | "
            f"Error (L-R): {error:7.2f} | Correction: {correction:6d} | Deriv: {derivative:7.2f} | "
            f"Integral: {integral:8.2f} | PWM L/R: {pwm_left:5d}/{pwm_right:5d} | Correction side: {correction_side}"
        )
        utime.sleep(UPDATE_INTERVAL)

    print("Done. Stopping motors.")
    PWM1.duty_u16(0)
    PWM2.duty_u16(0)

    # Clean up encoders (optional)
    enc_right.deinit()
    enc_left.deinit()
