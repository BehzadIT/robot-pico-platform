# track_auto_balance2.py
import utime
from machine import Pin, PWM
from services.dual_rpm_pio_test import PIOQuadratureCounter, read_rpms

def sync_tracks_pid(
    measured_rpm_left,
    measured_rpm_right,
    prev_pwm_left,
    prev_pwm_right,
    prev_error,
    integral,
    dt,
    kp=7.0,
    ki=1.2,
    kd=0.5,
    min_pwm=0,
    max_pwm=65535,
    integral_min=-50.0,
    integral_max=50.0,
):
    integral_locked = None  # Track if anti-windup is active

    if measured_rpm_left < measured_rpm_right:
        error = measured_rpm_left - measured_rpm_right
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
    elif measured_rpm_right < measured_rpm_left:
        error = measured_rpm_right - measured_rpm_left
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
    return pwm_left, pwm_right, prev_error, integral, error, correction, derivative, correction_side, integral_locked



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
    enc_right = PIOQuadratureCounter(0, 10, 11)
    enc_left  = PIOQuadratureCounter(1, 20, 21)

    # Start both tracks forward
    DIR1.value(1)
    DIR2.value(1)

    NUM_STEPS = 100
    UPDATE_INTERVAL = 0.05  # 50ms
    pwm_left = 65535
    pwm_right = 65535
    prev_error = 0
    integral = 0

    for step in range(NUM_STEPS):
        measured_rpm_right, measured_rpm_left = read_rpms(
            enc_right, enc_left, interval_sec=UPDATE_INTERVAL
        )
        (pwm_left, pwm_right, prev_error, integral,
         error, correction, derivative, correction_side, integral_locked) = sync_tracks_pid(
            measured_rpm_left, measured_rpm_right,
            pwm_left, pwm_right,
            prev_error, integral,
            dt=UPDATE_INTERVAL,
            kp=7.0, ki=1.2, kd=0.5,
            integral_min=-50.0, integral_max=50.0
        )
        PWM1.duty_u16(int(pwm_left))
        PWM2.duty_u16(int(pwm_right))
        lock_str = f" | Integral lock: {integral_locked.upper()}" if integral_locked else ""
        print(
            f"Step {step:3d} | RPM L/R: {measured_rpm_left:7.2f}/{measured_rpm_right:7.2f} | "
            f"Error (L-R): {error:7.2f} | Correction: {correction:6d} | Deriv: {derivative:7.2f} | "
            f"Integral: {integral:8.2f} | PWM L/R: {pwm_left:5d}/{pwm_right:5d} | "
            f"Correction side: {correction_side}{lock_str}"
        )
        utime.sleep(UPDATE_INTERVAL)

    print("Done. Stopping motors.")
    PWM1.duty_u16(0)
    PWM2.duty_u16(0)
    enc_right.deinit()
    enc_left.deinit()