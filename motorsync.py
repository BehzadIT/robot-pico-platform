from log import *
import time
from machine import Pin, PWM

# === Constants ===
ENCODER_CPR = 64
GEAR_RATIO = 30
TICKS_PER_REV = ENCODER_CPR  # Already 4× decoded from Pololu spec

# === Motor Control Pins ===
PWM1 = PWM(Pin(8))   # Motor 1 PWM
DIR1 = Pin(9, Pin.OUT)  # Motor 1 Direction
PWM2 = PWM(Pin(6))   # Motor 2 PWM
DIR2 = Pin(7, Pin.OUT)  # Motor 2 Direction

PWM1.freq(1000)
PWM2.freq(1000)
DIR1.value(1)
DIR2.value(1)
PWM1.duty_u16(65535)
PWM2.duty_u16(65535)

# === Encoder Pins ===
A1 = Pin(10, Pin.IN, Pin.PULL_UP)  # Motor 1 Encoder A
B1 = Pin(11, Pin.IN, Pin.PULL_UP)  # Motor 1 Encoder B
A2 = Pin(20, Pin.IN, Pin.PULL_UP)  # Motor 2 Encoder A
B2 = Pin(21, Pin.IN, Pin.PULL_UP)  # Motor 2 Encoder B

# === State Tracking ===
ticks1 = 0
last_A1 = A1.value()
last_B1 = B1.value()

ticks2 = 0
last_A2 = A2.value()
last_B2 = B2.value()

# === Quadrature Lookup Table ===
quad_table = [
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
]

# === Encoder Interrupt Handlers ===
def update_encoder1(pin):
    global ticks1, last_A1, last_B1
    A = A1.value()
    B = B1.value()
    state = (last_A1 << 3) | (last_B1 << 2) | (A << 1) | B
    ticks1 += quad_table[state & 0x0F]
    last_A1 = A
    last_B1 = B

def update_encoder2(pin):
    global ticks2, last_A2, last_B2
    A = A2.value()
    B = B2.value()
    state = (last_A2 << 3) | (last_B2 << 2) | (A << 1) | B
    ticks2 += quad_table[state & 0x0F]
    last_A2 = A
    last_B2 = B

# === Attach Interrupts to A and B Pins ===
A1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder1)
B1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder1)
A2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder2)
B2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder2)

# === Test Run ===
logi("Running both motors at full speed for 1 second...")
ticks1 = 0
ticks2 = 0
start = time.ticks_ms()
time.sleep(1.0)
elapsed = time.ticks_diff(time.ticks_ms(), start)

PWM1.duty_u16(0)
PWM2.duty_u16(0)

# === RPM Calculations ===
revs1 = ticks1 / TICKS_PER_REV
revs2 = ticks2 / TICKS_PER_REV
motor_rpm1 = revs1 / (elapsed / 60000)
motor_rpm2 = revs2 / (elapsed / 60000)
output_rpm1 = motor_rpm1 / GEAR_RATIO
output_rpm2 = motor_rpm2 / GEAR_RATIO

# === Results ===
logi("\n=== DUAL MOTOR RPM TEST RESULTS ===")
logd(f"Motor 1: Ticks={ticks1}, Motor RPM={motor_rpm1:.1f}, Output RPM={output_rpm1:.1f}")
logd(f"Motor 2: Ticks={ticks2}, Motor RPM={motor_rpm2:.1f}, Output RPM={output_rpm2:.1f}")
