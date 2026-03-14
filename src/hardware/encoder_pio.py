"""Low-level PIO quadrature helper for one encoder.

This file is only the building block for talking to a single quadrature encoder
through one PIO state machine.

It does not own the robot's full encoder setup.
It does not decide which pins the drivetrain uses.
It must not create global encoder instances at import time.

The higher-level drivetrain encoder owner lives in `encoder_subsystem.py`.
"""

import array
import rp2
from machine import Pin


class PIOQuadratureCounter:
    """PIO-backed counter for one encoder.

    This class is intentionally small:
    - one state machine
    - one encoder pin pair
    - read/reset/restart/deinit operations

    The robot should normally create these through the drivetrain encoder
    subsystem instead of directly from the control loop.
    """

    @rp2.asm_pio(autopush=False, autopull=False)
    def counter():
        jmp("update")
        jmp("decrement")
        jmp("increment")
        jmp("update")

        jmp("increment")
        jmp("update")
        jmp("update")
        jmp("decrement")

        jmp("decrement")
        jmp("update")
        jmp("update")
        jmp("increment")

        jmp("update")
        jmp("increment")

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

    def __init__(self, state_machine_id, pin_a, pin_b, freq=125_000_000):
        if pin_b != pin_a + 1:
            raise ValueError("pin_b must be pin_a + 1 for PIO quadrature")

        self.state_machine_id = state_machine_id
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.freq = freq
        self._buf = array.array("i", [0])

        Pin(pin_a, Pin.IN, Pin.PULL_UP)
        Pin(pin_b, Pin.IN, Pin.PULL_UP)

        self.sm = rp2.StateMachine(
            state_machine_id,
            self.counter,
            freq=freq,
            in_base=Pin(pin_a),
        )
        self.sm.active(1)
        self.set_zero()

    def read(self):
        self.sm.put(1)
        self.sm.get(self._buf)
        return self._buf[0]

    def set_zero(self):
        self.sm.exec("set(y, 0)")

    def restart(self):
        self.sm.active(0)
        self.sm.restart()
        self.sm.active(1)
        self.set_zero()

    def deinit(self):
        self.sm.active(0)
