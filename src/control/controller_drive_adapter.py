"""Compatibility shim for controller status access.

The live Pico controller pipeline now runs entirely inside
`joystick_uart_receiver.py` so the RP2040 uses only one extra MicroPython
worker. This module remains as a stable import location for status access while
runtime thread ownership lives in the transport-owned controller pipeline.
"""

from src.transport.joystick_uart_receiver import controller_drive_status_snapshot


class ControllerDriveAdapter:
    """Compatibility wrapper retained for existing imports."""

    def start(self):
        return True

    def status_snapshot(self):
        return controller_drive_status_snapshot()


controller_drive_adapter = ControllerDriveAdapter()
