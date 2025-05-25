from machine import Pin, PWM
import time

# === Constants ===
ENCODER_CPR = 64
GEAR_RATIO = 30
TICKS_PER_REV = ENCODER_CPR // 4  # 1x decoding: only A rising edges

# === Motor Control Pins ===
PWM1 = PWM(Pin(6))   # Motor 1 PWM
DIR1 = Pin(7, Pin.OUT)
PWM2 = PWM(Pin(8))   # Motor 2 PWM
DIR2 = Pin(9, Pin.OUT)

PWM1.freq(1000)
PWM2.freq(1000)
DIR1.value(1)
DIR2.value(1)
PWM1.duty_u16(65535)
PWM2.duty_u16(65535)

# === Encoder Pins (1x decoding) ===
A1 = Pin(10, Pin.IN, Pin.PULL_UP)  # Motor 1 A
B1 = Pin(11, Pin.IN, Pin.PULL_UP)  # Motor 1 B
A2 = Pin(20, Pin.IN, Pin.PULL_UP)  # Motor 2 A
B2 = Pin(21, Pin.IN, Pin.PULL_UP)  # Motor 2 B

# === Tick Counters ===
ticks1 = 0
ticks2 = 0

# === Interrupt Handlers (rising edge only) ===
def update_encoder1(pin):
    global ticks1
    ticks1 += 1 if B1.value() else -1

def update_encoder2(pin):
    global ticks2
    ticks2 += 1 if B2.value() else -1

# === Attach IRQs for A channels only (1× decoding) ===
A1.irq(trigger=Pin.IRQ_RISING, handler=update_encoder1)
A2.irq(trigger=Pin.IRQ_RISING, handler=update_encoder2)

# === Run Test ===
print("Running both motors at full speed for 1 second...")
ticks1 = 0
ticks2 = 0
start = time.ticks_ms()
time.sleep(1.0)
elapsed = time.ticks_diff(time.ticks_ms(), start)

PWM1.duty_u16(0)
PWM2.duty_u16(0)

# === RPM Calculation ===
revs1 = ticks1 / TICKS_PER_REV
revs2 = ticks2 / TICKS_PER_REV
motor_rpm1 = revs1 / (elapsed / 60000)
motor_rpm2 = revs2 / (elapsed / 60000)
output_rpm1 = motor_rpm1 / GEAR_RATIO
output_rpm2 = motor_rpm2 / GEAR_RATIO

# === Display Results ===
print("\n=== DUAL MOTOR RPM TEST RESULTS (1x DECODING) ===")
print(f"Motor 1: Ticks={ticks1}, Motor RPM={motor_rpm1:.1f}, Output RPM={output_rpm1:.1f}")
print(f"Motor 2: Ticks={ticks2}, Motor RPM={motor_rpm2:.1f}, Output RPM={output_rpm2:.1f}")
