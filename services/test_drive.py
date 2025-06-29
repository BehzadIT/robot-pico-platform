from machine import Pin, PWM
import time

# === Motor driver PWM & direction pins ===
PWM1 = PWM(Pin(8))   # Right track
DIR1 = Pin(9, Pin.OUT)
PWM2 = PWM(Pin(6))   # Left track
DIR2 = Pin(7, Pin.OUT)

PWM1.freq(1000)
PWM2.freq(1000)

# === Encoder pins ===
ENC1_A = Pin(10, Pin.IN, Pin.PULL_UP)  # Right track
ENC1_B = Pin(11, Pin.IN, Pin.PULL_UP)
ENC2_A = Pin(20, Pin.IN, Pin.PULL_UP)  # Left track
ENC2_B = Pin(21, Pin.IN, Pin.PULL_UP)

# === Encoder tick counters ===
ticks_right = 0
ticks_left = 0

def right_encoder_irq(pin):
    global ticks_right
    A = ENC1_A.value()
    B = ENC1_B.value()
    # Quadrature decode (simplified, for direction):
    if A == B:
        ticks_right += 1
    else:
        ticks_right -= 1

def left_encoder_irq(pin):
    global ticks_left
    A = ENC2_A.value()
    B = ENC2_B.value()
    if A == B:
        ticks_left += 1
    else:
        ticks_left -= 1

ENC1_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=right_encoder_irq)
ENC1_B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=right_encoder_irq)
ENC2_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=left_encoder_irq)
ENC2_B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=left_encoder_irq)

# === Constants ===
TICKS_PER_OUTPUT_REV = 1920  # Pololu 37D 30:1 @ output shaft

def get_rpm_left(interval_sec=0.05):
    global ticks_left
    start_ticks = ticks_left
    time.sleep(interval_sec)
    end_ticks = ticks_left
    delta_ticks = end_ticks - start_ticks
    rpm = (delta_ticks / TICKS_PER_OUTPUT_REV) / interval_sec * 60
    return rpm

def get_rpm_right(interval_sec=0.05):
    global ticks_right
    start_ticks = ticks_right
    time.sleep(interval_sec)
    end_ticks = ticks_right
    delta_ticks = end_ticks - start_ticks
    rpm = (delta_ticks / TICKS_PER_OUTPUT_REV) / interval_sec * 60
    return rpm

# === Correction function ===
def sync_tracks_straight(
    target_rpm: float,
    measured_rpm_left: float,
    measured_rpm_right: float,
    prev_pwm_left: int,
    prev_pwm_right: int,
    kp: float = 0.5
) -> tuple[int, int]:
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

def test_sync_tracks_full_speed(runtime_sec=5.0, update_interval=0.05):
    max_rpm = 330
    target_rpm = max_rpm
    DIR1.value(1)
    DIR2.value(1)
    pwm_left = 65535
    pwm_right = 65535
    start_time = time.ticks_ms()
    print("Starting test: 5s at full speed with track synchronization.")

    global ticks_left, ticks_right  # So we can access these

    while time.ticks_diff(time.ticks_ms(), start_time) < runtime_sec * 1000:
        # Snapshot tick counters at start
        start_ticks_left = ticks_left
        start_ticks_right = ticks_right

        # Wait for the interval
        time.sleep(update_interval)

        # Snapshot tick counters at end
        end_ticks_left = ticks_left
        end_ticks_right = ticks_right

        # Calculate deltas over the interval
        delta_left = end_ticks_left - start_ticks_left
        delta_right = end_ticks_right - start_ticks_right

        # Calculate RPMs
        rpm_left = (delta_left / TICKS_PER_OUTPUT_REV) / update_interval * 60
        rpm_right = (delta_right / TICKS_PER_OUTPUT_REV) / update_interval * 60

        # Correction step
        pwm_left, pwm_right = sync_tracks_straight(
            target_rpm, rpm_left, rpm_right, pwm_left, pwm_right, kp=0.5
        )

        # Apply PWM
        PWM2.duty_u16(pwm_left)
        PWM1.duty_u16(pwm_right)

        # Print out all info, with precise RPMs and delta
        print(
            "Time: {:.2f}s | PWM L: {:5d} | RPM L: {:8.3f} | "
            "PWM R: {:5d} | RPM R: {:8.3f} | Delta RPM (L-R): {:+8.3f}".format(
                time.ticks_diff(time.ticks_ms(), start_time) / 1000,
                pwm_left, rpm_left,
                pwm_right, rpm_right,
                rpm_left - rpm_right
            )
        )

    print("Test complete. Stopping motors.")
    PWM1.duty_u16(0)
    PWM2.duty_u16(0)


# === Run the test ===
# test_sync_tracks_full_speed(runtime_sec=5.0, update_interval=0.05)
