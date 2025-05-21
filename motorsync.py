from machine import Pin, PWM
import time

# === Motor Pins ===
PWM1 = PWM(Pin(8))
DIR1 = Pin(9, Pin.OUT)
PWM2 = PWM(Pin(6))
DIR2 = Pin(7, Pin.OUT)

PWM1.freq(1000)
PWM2.freq(1000)

# === Encoder Pins ===
ENC1_A = Pin(10, Pin.IN)
ENC1_B = Pin(11, Pin.IN)
ENC2_A = Pin(20, Pin.IN)
ENC2_B = Pin(21, Pin.IN)

# === Constants ===
TICKS_PER_REV = 546  # 13 PPR x 42 Gear Ratio
MAX_PWM = 65535
Kp = 0.9
rpm_to_pwm_scale = 546
min_operating_pwm = 10000
ramp_duration = 10
ramp_steps = 100
step_delay = ramp_duration / ramp_steps

# === Encoder Tick Counters ===
encoder1_ticks = 0
encoder2_ticks = 0

# === Interrupt Handlers ===
def encoder1_handler(pin):
    global encoder1_ticks
    if ENC1_B.value() == 0:
        encoder1_ticks += 1
    else:
        encoder1_ticks -= 1

def encoder2_handler(pin):
    global encoder2_ticks
    if ENC2_B.value() == 0:
        encoder2_ticks += 1
    else:
        encoder2_ticks -= 1

ENC1_A.irq(trigger=Pin.IRQ_RISING, handler=encoder1_handler)
ENC2_A.irq(trigger=Pin.IRQ_RISING, handler=encoder2_handler)

# === Utility Functions ===
def set_motor(pwm_val, pwm_pin, dir_pin):
    dir_pin.value(pwm_val >= 0)
    pwm_pin.duty_u16(min(abs(int(pwm_val)), MAX_PWM))

def calculate_rpm(ticks, duration_ms):
    revs = ticks / TICKS_PER_REV
    minutes = duration_ms / 60000
    return revs / minutes

def adjust_pwm(target_rpm, current_rpm, current_pwm):
    error = target_rpm - current_rpm
    adjustment = Kp * error * rpm_to_pwm_scale
    new_pwm = current_pwm + adjustment
    return max(0, min(int(new_pwm), MAX_PWM))

# === Synchronized Balanced Ramp ===
def synchronized_balanced_ramp(direction=1):
    pwm1_val = 0
    pwm2_val = 0

    for step in range(ramp_steps + 1):
        if direction >= 0:
            base_pwm = int((step / ramp_steps) * (MAX_PWM - min_operating_pwm)) + min_operating_pwm
        else:
            base_pwm = -int((step / ramp_steps) * (MAX_PWM - min_operating_pwm)) - min_operating_pwm

        # Set motors BEFORE resetting ticks
        set_motor(base_pwm, PWM1, DIR1)
        set_motor(base_pwm, PWM2, DIR2)

        global encoder1_ticks, encoder2_ticks
        encoder1_ticks = 0
        encoder2_ticks = 0
        start_time = time.ticks_ms()

        time.sleep(1.0)

        elapsed = time.ticks_diff(time.ticks_ms(), start_time)
        rpm1 = calculate_rpm(encoder1_ticks, elapsed)
        rpm2 = calculate_rpm(encoder2_ticks, elapsed)

        target_rpm = min(rpm1, rpm2)
        pwm1_val = adjust_pwm(target_rpm, rpm1, base_pwm)
        pwm2_val = adjust_pwm(target_rpm, rpm2, base_pwm)

        set_motor(pwm1_val, PWM1, DIR1)
        set_motor(pwm2_val, PWM2, DIR2)

        print("Step: {:3d} | Time: {:4d}ms | Target: {:6.1f} | RPM1: {:6.1f} | RPM2: {:6.1f} | PWM1: {:5d} | PWM2: {:5d}".format(
            step, elapsed, target_rpm, rpm1, rpm2, pwm1_val, pwm2_val
        ))

# === Main Test ===
def run_test():
    print("Starting forward ramp...")
    synchronized_balanced_ramp(direction=1)

    print("Holding for 2 seconds...")
    set_motor(0, PWM1, DIR1)
    set_motor(0, PWM2, DIR2)
    time.sleep(2)

    print("Starting reverse ramp...")
    synchronized_balanced_ramp(direction=-1)

    print("Stopping motors.")
    set_motor(0, PWM1, DIR1)
    set_motor(0, PWM2, DIR2)
