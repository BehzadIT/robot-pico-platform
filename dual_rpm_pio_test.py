
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
    #  the code must be loaded at address 0, because it uses computed jumps origin 0

    #  the code works by running a loop that continuously shifts the 2 phase pins into
    #  ISR and looks at the lower 4 bits to do a computed jump to an instruction that
    #  does the proper "do nothing" | "increment" | "decrement" action for that pin
    #  state change (or no change)

    #  ISR holds the last state of the 2 pins during most of the code. The Y register
    #  keeps the current encoder count and is incremented / decremented according to
    #  the steps sampled

    #  the program keeps trying to write the current count to the RX FIFO without
    #  blocking. To read the current count, the user code must drain the FIFO first
    #  and wait for a fresh sample (takes ~4 SM cycles on average). The worst case
    #  sampling loop takes 10 cycles, so this program is able to read step rates up
    #  to sysclk / 10  (e.g., sysclk 125MHz, max step rate = 12.5 Msteps/sec)

    @rp2.asm_pio(autopush=False, autopull=False)
    def counter():
        # --- Quadrature decoder state table (4x4 = 16 entries) ---
        # Each 'jmp' handles a transition from previous to current encoder state.
        # The action is: update (no change), increment, or decrement the counter.
        jmp("update")    # 00 -> 00: no change, ISR: 0b0000 -> line 0
        jmp("decrement") # 00 -> 01: count down, ISR: 0b0001 -> line 1
        jmp("increment") # 00 -> 10: count up, ISR: 0b0010 -> line 2
        jmp("update")    # 00 -> 11: no change, ISR: 0b0011 -> line 3

        jmp("increment") # 01 -> 00: count up, ISR: 0b0100 -> line 4
        jmp("update")    # 01 -> 01: no change, ISR: 0b0101 -> line 5
        jmp("update")    # 01 -> 10: no change, ISR: 0b0110 -> line 6
        jmp("decrement") # 01 -> 11: count down, ISR: 0b0111 -> line 7

        jmp("decrement") # 10 -> 00: count down, ISR: 0b1000 -> line 8
        jmp("update")    # 10 -> 01: no change, ISR: 0b1001 -> line 9
        jmp("update")    # 10 -> 10: no change, ISR: 0b1010 -> line 10
        jmp("increment") # 10 -> 11: count up, ISR: 0b1011 -> line 11

        jmp("update")    # 11 -> 00: no change, ISR: 0b1100 -> line 12
        jmp("increment") # 11 -> 01: count up, ISR: 0b1101 -> line 13

        # --- Decrement handler ---
        # to save instruction space, we used the actual "decrement" label instead jmp("update")
        label("decrement")   # 11 -> 10: no change, ISR: 0b1110 -> line 14
        # to save instruction space, we do decrement and a branch in one opcode
        jmp(y_dec, "update") # 11 -> 11: no change, ISR: 0b1111 -> line 15

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
enc1 = PIOQuadratureCounter(0, 10, 11)
enc2 = PIOQuadratureCounter(1, 20, 21)

# === Reset before run (optional if already reset in constructor) ===
enc1.set_zero()
enc2.set_zero()


def read_rpms(enc1, enc2, interval_sec=0.05, cpr=ENCODER_CPR, gear_ratio=GEAR_RATIO):
    """
    Reads and returns the output shaft RPMs for both motors over a given interval.
    :param enc1, enc2: PIOQuadratureCounter instances
    :param interval_sec: Measurement interval (e.g. 0.05s)
    :return: (rpm_right, rpm_left)
    """
    start1 = enc1.read()
    start2 = enc2.read()
    utime.sleep(interval_sec)
    end1 = enc1.read()
    end2 = enc2.read()
    delta1 = end1 - start1
    delta2 = end2 - start2
    # Output shaft RPM (full quadrature)
    rpm1 = (delta1 / (cpr * gear_ratio)) / interval_sec * 60
    rpm2 = (delta2 / (cpr * gear_ratio)) / interval_sec * 60
    return rpm1, rpm2  # right, left


def read_rpm(enc, interval_sec=0.05, cpr=ENCODER_CPR, gear_ratio=GEAR_RATIO):
    """
    Reads and returns the output shaft RPM for a single motor over a given interval.
    :param enc: PIOQuadratureCounter instance
    :param interval_sec: Measurement interval (e.g. 0.05s)
    :return: rpm
    """
    start = enc.read()
    utime.sleep(interval_sec)
    end = enc.read()
    delta = end - start
    # Output shaft RPM (full quadrature)
    rpm = (delta / (cpr * gear_ratio)) / interval_sec * 60
    return rpm


def read_left_motor_rpm(interval_sec=0.05):
    return read_rpm(enc2, interval_sec, cpr=ENCODER_CPR, gear_ratio=GEAR_RATIO)

def read_right_motor_rpm(interval_sec=0.05):
    return read_rpm(enc1, interval_sec, cpr=ENCODER_CPR, gear_ratio=GEAR_RATIO)

# === Motor Pins ===
PWM1 = PWM(Pin(7)); DIR1 = Pin(6, Pin.OUT)
PWM2 = PWM(Pin(9)); DIR2 = Pin(8, Pin.OUT)
PWM1.freq(1000); PWM2.freq(1000)

# === Encoder Instances ===

def start_test(run_time_secs=5):
    PWM_MAX = 65535  # Max duty cycle for PWM
    DIR1.value(0)
    DIR2.value(0)
    PWM1.duty_u16(PWM_MAX)
    PWM2.duty_u16(PWM_MAX)

    print("Running motors and measuring RPM using PIO encoder...")
    utime.sleep(run_time_secs)

    PWM1.duty_u16(0)
    PWM2.duty_u16(0)

    # === Read and Reset Encoder Ticks ===
    ticks1 = enc1.read(); enc1.set_zero()
    ticks2 = enc2.read(); enc2.set_zero()

    motor_rpm1 = (ticks1 / ENCODER_CPR) * (60 / run_time_secs)
    motor_rpm2 = (ticks2 / ENCODER_CPR) * (60 / run_time_secs)
    output_rpm1 = motor_rpm1 / GEAR_RATIO
    output_rpm2 = motor_rpm2 / GEAR_RATIO

    print("\n=== FINAL RPM RESULTS (WITH RESET) ===")
    print(f"Motor 1: Ticks={ticks1}, Motor RPM={motor_rpm1:.1f}, Output RPM={output_rpm1:.1f}")
    print(f"Motor 2: Ticks={ticks2}, Motor RPM={motor_rpm2:.1f}, Output RPM={output_rpm2:.1f}")

    enc1.deinit()
    enc2.deinit()
    gc.collect()
