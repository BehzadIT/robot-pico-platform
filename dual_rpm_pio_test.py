
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
        # --- Quadrature decoder state table (4x4 = 16 entries) ---
        # Each 'jmp' handles a transition from previous to current encoder state.
        # The action is: update (no change), increment, or decrement the counter.
        jmp("update")    # 00 -> 00: no change
        jmp("decrement") # 00 -> 01: count down
        jmp("increment") # 00 -> 10: count up
        jmp("update")    # 00 -> 11: no change

        jmp("increment") # 01 -> 00: count up
        jmp("update")    # 01 -> 01: no change
        jmp("update")    # 01 -> 10: no change
        jmp("decrement") # 01 -> 11: count down

        jmp("decrement") # 10 -> 00: count down
        jmp("update")    # 10 -> 01: no change
        jmp("update")    # 10 -> 10: no change
        jmp("increment") # 10 -> 11: count up

        jmp("update")    # 11 -> 00: no change
        jmp("increment") # 11 -> 01: count up

        # --- Decrement handler ---
        label("decrement")
        jmp(y_dec, "update")  # Decrement y register (the counter), then go to update

        # --- Main update loop ---
        label("update")
        wrap_target()         # Mark start of main loop
        set(x, 0)             # Clear x register (used as a flag)
        pull(noblock)         # Try to pull a command from FIFO (non-blocking)
        mov(x, osr)           # Move pulled value to x
        mov(osr, isr)         # Save current isr to osr for later
        jmp(not_x, "sample_pins") # If x==0, no read command, go sample pins
        mov(isr, y)           # If x!=0, move counter (y) to isr
        push()                # Push isr (counter value) to FIFO for CPU to read

        # --- Sample encoder pins and update state ---
        label("sample_pins")
        mov(isr, null)        # Clear isr
        in_(osr, 2)           # Shift in previous state (2 bits) from osr
        in_(pins, 2)          # Shift in current state (2 bits) from pins
        mov(pc, isr)          # Jump to correct state handler based on 4-bit value

        # --- Increment handler ---
        label("increment")
        mov(x, invert(y))     # Invert y, store in x
        jmp(x_dec, "increment2") # Decrement x, jump if not zero
        label("increment2")
        mov(y, invert(x))     # Invert x, store in y (effectively y++)
        wrap()                # End of main loop

        # --- Padding for timing ---
        nop()
        nop()
        nop()

    def __init__(self, pio_id, pin_a, pin_b):
        # Ensure pins are consecutive (required by PIO)
        if pin_b != pin_a + 1:
            raise Exception("pin_b must be pin_a + 1")
        # Configure pins as inputs with pull-ups
        Pin(pin_a, Pin.IN, Pin.PULL_UP)
        Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self._pio_id = pio_id  # Store PIO state machine ID
        self.buf = array.array('i', [0])  # Buffer for reading count
        # Create and start the PIO state machine for this encoder
        self.sm = rp2.StateMachine(pio_id, self.counter, freq=125_000_000, in_base=Pin(pin_a))
        self.sm.active(1)
        self.set_zero()  # Reset counter at initialization

    def read(self):
        # Request a read from the PIO program and return the current count
        self.sm.put(1)
        self.sm.get(self.buf)
        return self.buf[0]

    def set_zero(self):
        # Reset the internal tick counter to zero
        self.sm.exec("set(y, 0)")

    def deinit(self):
        # Deactivate the state machine and remove the PIO program
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

