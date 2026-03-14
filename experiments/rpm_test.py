from machine import Pin, PWM
import time

# === ENCODER AND MOTOR CONSTANTS ===
ENCODER_CPR = 64              # Pololu 37D with encoder: 64 counts/rev at full 4× decoding
GEAR_RATIO = 30               # Gearbox: 30:1
TICKS_PER_REV = ENCODER_CPR   # ✅ CPR already includes 4× decoding

# === MOTOR CONTROL SETUP ===
PWM1 = PWM(Pin(8))
DIR1 = Pin(9, Pin.OUT)
PWM1.freq(1000)
DIR1.value(1)                 # Set motor direction to forward
PWM1.duty_u16(65535)          # Full speed

# === ENCODER PINS (QUADRATURE A + B) ===
A = Pin(20, Pin.IN, Pin.PULL_UP)
B = Pin(21, Pin.IN, Pin.PULL_UP)

# === QUADRATURE STATE ===
last_A = A.value()
last_B = B.value()
ticks = 0

# === QUADRATURE TRANSITION TABLE (4x decoding) ===
quad_table = [
     0,  -1,   1,   0,
     1,   0,   0,  -1,
    -1,   0,   0,   1,
     0,   1,  -1,   0
]

# === INTERRUPT HANDLER ===
def update_encoder(pin):
    global last_A, last_B, ticks
    A_val = A.value()
    B_val = B.value()
    state = (last_A << 3) | (last_B << 2) | (A_val << 1) | B_val
    ticks += quad_table[state & 0x0F]
    last_A = A_val
    last_B = B_val

# === ATTACH INTERRUPTS TO BOTH CHANNELS ===
A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder)
B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder)

from log import *

# === TEST RUN ===
logi("Running motor at full speed for 1 second...")
ticks = 0
start_time = time.ticks_ms()
time.sleep(1.0)
elapsed = time.ticks_diff(time.ticks_ms(), start_time)

# Stop motor
PWM1.duty_u16(0)

# === CALCULATE RPM ===
revs = ticks / TICKS_PER_REV
motor_rpm = revs / (elapsed / 60000)
output_rpm = motor_rpm / GEAR_RATIO

# === REPORT RESULTS ===
logi("\n=== RPM TEST RESULTS ===")
logd(f"Ticks counted:      {ticks}")
logd(f"Elapsed time (ms):  {elapsed}")
logi(f"Motor RPM:          {motor_rpm:.1f}")
logi(f"Output Shaft RPM:   {output_rpm:.1f}")
