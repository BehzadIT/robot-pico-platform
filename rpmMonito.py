from machine import Pin, Timer
import time

# === Motor 1 Encoder (Right Motor) ===
enc1_a = Pin(10, Pin.IN, Pin.PULL_UP)  # Signal A
enc1_b = Pin(11, Pin.IN, Pin.PULL_UP)  # Signal B
pos1 = 0

# === Motor 2 Encoder (Left Motor) ===
enc2_a = Pin(20, Pin.IN, Pin.PULL_UP)   # Signal A
enc2_b = Pin(21, Pin.IN, Pin.PULL_UP)   # Signal B
pos2 = 0

# === Correct CPR for rising-edge only counting ===
# Pololu 37D = 64 CPR at motor shaft × 30:1 gearbox = 1920 (full)
# Only 1 edge per cycle = 1920 / 4 = 480
COUNTS_PER_REV = 480

# === Motor 1 Interrupt ===
def enc1_irq(pin):
    global pos1
    if enc1_a.value() == 1:  # Rising edge
        if enc1_b.value() == 0:
            pos1 += 1
        else:
            pos1 -= 1

# === Motor 2 Interrupt ===
def enc2_irq(pin):
    global pos2
    if enc2_a.value() == 1:  # Rising edge
        if enc2_b.value() == 0:
            pos2 += 1
        else:
            pos2 -= 1

# === Attach interrupts on A channel rising only ===
enc1_a.irq(trigger=Pin.IRQ_RISING, handler=enc1_irq)
enc2_a.irq(trigger=Pin.IRQ_RISING, handler=enc2_irq)

# === Timing tracker ===
last_time = time.ticks_ms()

# === Print RPM every 0.5 sec ===
def print_rpm(timer):
    global pos1, pos2, last_time

    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time)
    last_time = now

    ticks1 = pos1
    ticks2 = pos2
    pos1 = 0
    pos2 = 0

    from log import logd
    rpm1 = (ticks1 / COUNTS_PER_REV) * 60000 / dt if dt > 0 else 0
    rpm2 = (ticks2 / COUNTS_PER_REV) * 60000 / dt if dt > 0 else 0

    logd(f"Motor 1 RPM: {round(rpm1, 2)} | Motor 2 RPM: {round(rpm2, 2)}")

# === Start timer at 2 Hz (every 0.5s) ===
rpm_timer = Timer()
rpm_timer.init(freq=2, mode=Timer.PERIODIC, callback=print_rpm)

# === Keep running ===
while True:
    time.sleep(1)
