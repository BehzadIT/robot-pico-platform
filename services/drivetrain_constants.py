"""Structured hardware and timing constants for the drivetrain firmware.

This module centralizes the Pico pin mapping, encoder geometry, and control
timing defaults used by the robot's tracked drivetrain.

The intent is to keep board-specific wiring and fixed hardware facts in one
place while leaving runtime state and control behavior in the controller and
encoder subsystem modules.
"""


class MotorPins:
    """Cytron MDD10A motor-driver wiring on the Pico."""

    RIGHT_PWM = 7
    RIGHT_DIR = 6
    LEFT_PWM = 9
    LEFT_DIR = 8


class MotorLimits:
    """PWM and motor-speed constants for the drive outputs."""

    RPM_MAX = 330
    PWM_MIN = 4000
    PWM_MAX = 65535
    # 20 kHz keeps PWM above the audible range for most users and is a
    # reasonable operating point for the Cytron driver used in this robot.
    PWM_FREQ = 20_000


class EncoderPins:
    """PIO state-machine assignment and GPIO mapping for the two encoders."""

    RIGHT_SM_ID = 0
    RIGHT_PIN_A = 10
    RIGHT_PIN_B = 11
    LEFT_SM_ID = 1
    LEFT_PIN_A = 20
    LEFT_PIN_B = 21


class EncoderGeometry:
    """Physical encoder conversion constants.

    `ENCODER_CPR` is the fully decoded quadrature count at the motor shaft.
    Multiply by `GEAR_RATIO` to derive output-shaft counts per revolution.
    """

    ENCODER_CPR = 64
    GEAR_RATIO = 30


class ControlTiming:
    """Default drivetrain control-loop timing."""

    CONTROL_PERIOD_US = 20_000
    SUMMARY_PERIOD_MS = 1_000


class SafetyTiming:
    """Drivetrain safety timings.

    `COMMAND_TIMEOUT_MS` is intentionally much shorter than a full network
    reconnect cycle so a missed stop or stalled command stream drops motor
    output quickly. `WDT_TIMEOUT_MS` is longer because it is only a last-resort
    backstop for full worker/interpreter wedges.
    """

    COMMAND_TIMEOUT_MS = 500
    WDT_TIMEOUT_MS = 2_000


class SteeringLimits:
    """Steering-angle semantics shared across transport, UI, and control.

    Positive angle means steer right.
    Negative angle means steer left.
    """

    ANGLE_MIN = -90
    ANGLE_MAX = 90
    ANGLE_DEFAULT = 0
