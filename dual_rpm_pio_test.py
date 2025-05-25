
import machine
import utime
import rp2
from machine import Pin, PWM
import array
import gc

# base on official Raspberry Pi Pico PIO example for quadrature encoder counting
# https://github.com/raspberrypi/pico-examples/blob/master/pio/quadrature_encoder/quadrature_encoder.pio
# === PIO-based Quadrature Encoder Counter with Reset ===
class PIOQuadratureCounter:
    @rp2.asm_pio(autopush=False, autopull=False)
    def counter():
        jmp("update")    # 00 -> 00
        jmp("decrement") # 00 -> 01
        jmp("increment") # 00 -> 10
        jmp("update")    # 00 -> 11

        jmp("increment") # 01 -> 00
        jmp("update")    # 01 -> 01
        jmp("update")    # 01 -> 10
        jmp("decrement") # 01 -> 11

        jmp("decrement") # 10 -> 00
        jmp("update")    # 10 -> 01
        jmp("update")    # 10 -> 10
        jmp("increment") # 10 -> 11

        jmp("update")    # 11 -> 00
        jmp("increment") # 11 -> 01
        label("decrement")
        jmp(y_dec, "update")
        label("update")
        wrap_target()
        set(x, 0)
        pull(noblock)
        mov(x, osr)
        mov(osr, isr)
        jmp(not_x, "sample_pins")
        mov(isr, y)
        push()
        label("sample_pins")
        mov(isr, null)
        in_(osr, 2)
        in_(pins, 2)
        mov(pc, isr)
        label("increment")
        mov(x, invert(y))
        jmp(x_dec, "increment2")
        label("increment2")
        mov(y, invert(x))
        wrap()
        nop()
        nop()
        nop()

    def __init__(self, pio_id, pin_a, pin_b):
        if pin_b != pin_a + 1:
            raise Exception("pin_b must be pin_a + 1")
        Pin(pin_a, Pin.IN, Pin.PULL_UP)
        Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self._pio_id = pio_id
        self.buf = array.array('i', [0])
        self.sm = rp2.StateMachine(pio_id, self.counter, freq=125_000_000, in_base=Pin(pin_a))
        self.sm.active(1)
        self.set_zero()  # Reset counter at initialization

    def read(self):
        self.sm.put(1)
        self.sm.get(self.buf)
        return self.buf[0]

    def set_zero(self):
        self.sm.exec("set(y, 0)")  # Reset internal tick counter

    def deinit(self):
        self.sm.active(0)
        rp2.PIO(self._pio_id).remove_program()

# === Constants ===
ENCODER_CPR = 64
GEAR_RATIO = 30

# === Motor Pins ===
PWM1 = PWM(Pin(6)); DIR1 = Pin(7, Pin.OUT)
PWM2 = PWM(Pin(8)); DIR2 = Pin(9, Pin.OUT)
PWM1.freq(1000); PWM2.freq(1000)

# === Encoder Instances ===
enc1 = PIOQuadratureCounter(0, 10, 11)
enc2 = PIOQuadratureCounter(1, 20, 21)

# === Reset before run (optional if already reset in constructor) ===
enc1.set_zero()
enc2.set_zero()

# === Start Motors ===
DIR1.value(1); DIR2.value(1)
PWM1.duty_u16(65535); PWM2.duty_u16(65535)

print("Running motors and measuring RPM using PIO encoder...")
utime.sleep(1.0)

PWM1.duty_u16(0)
PWM2.duty_u16(0)

# === Read and Reset Encoder Ticks ===
ticks1 = enc1.read(); enc1.set_zero()
ticks2 = enc2.read(); enc2.set_zero()

motor_rpm1 = (ticks1 / ENCODER_CPR) * 60
motor_rpm2 = (ticks2 / ENCODER_CPR) * 60
output_rpm1 = motor_rpm1 / GEAR_RATIO
output_rpm2 = motor_rpm2 / GEAR_RATIO

# === Display Results ===
print("\n=== FINAL RPM RESULTS (WITH RESET) ===")
print(f"Motor 1: Ticks={ticks1}, Motor RPM={motor_rpm1:.1f}, Output RPM={output_rpm1:.1f}")
print(f"Motor 2: Ticks={ticks2}, Motor RPM={motor_rpm2:.1f}, Output RPM={output_rpm2:.1f}")

# === Clean Up ===
enc1.deinit()
enc2.deinit()
gc.collect()

