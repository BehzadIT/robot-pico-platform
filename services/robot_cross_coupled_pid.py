# robot_cross_coupled_pid.py
# Differential drive robot cross-coupled PID controller for Raspberry Pi Pico (MicroPython)
# Hardware: Pololu 37D motors & encoders, Cytron MDD10A motor driver
# Dependencies: simple_pid.py in same directory or lib

from machine import Pin, PWM
import time
from services.simple_pid import PID
from services.dual_rpm_pio_test import read_rpms, PIOQuadratureCounter
from services.robot_drive_controller import driverController

# ----------------------------
# Logging Helper (improves future debugging and enables easy expansion)
# ----------------------------
def log(msg, *args):
    """Uniform log output for PID and hardware events."""
    try:
        print("[PID] " + (msg % args if args else msg))
    except Exception:
        print("[PID]", msg)

# ----------------------------
# Hardware Pin Setup - Pico to Cytron MDD10A, Pololu 37D
# ----------------------------
# Pico GPIO Pin mapping (adjust to your wiring as needed)
PWM_RIGHT = PWM(Pin(7))   # Right motor PWM (Cytron MDD10A PWM2 pin)
DIR_RIGHT = Pin(6, Pin.OUT)  # Right motor DIR (Cytron MDD10A DIR2 pin)
PWM_LEFT = PWM(Pin(9))    # Left motor PWM (Cytron MDD10A PWM1 pin)
DIR_LEFT = Pin(8, Pin.OUT)    # Left motor DIR (Cytron MDD10A DIR1 pin)
# For reversing, swap DIR logic as needed (see Cytron manual)

# Encoder pins are handled in your dual_rpm_pio_test.py module
# Ensure encoder A/B pins are consecutive for Pico PIO SM (hardware requirement)
enc_right = None  # Will be set per control thread run
enc_left = None

# ----------------------------
# Control & Normalization Constants
# ----------------------------
RPM_MIN = 0
RPM_MAX = 330  # Match your max tested output shaft RPM at 12V
PWM_MIN = 4000  # Avoids motor stalling/creep
PWM_MAX = 65535  # 16-bit PWM for full-range Cytron control
terminate = False

# ----------------------------
# PID Constants (Tune for your robot!)
# ----------------------------
PID_KP = 0.489
PID_KI = 0.92
PID_KD = 0

K_DELTA = 0.5  # Cross-coupling strength (affects turn sharpness)
PID_OUTPUT_MIN = 0.0
PID_OUTPUT_MAX = 1.0
PID_INTEGRAL_MIN = 0.0
PID_INTEGRAL_MAX = 1.0
PWM_FREQ = 20000  # 20kHz = silent and efficient on Cytron

OSCILLATION_THRESHOLD = 10000  # Used in TEST_MODE for stability testing

current_pwm_right = 0
current_pwm_left = 0
prev_pwm_left = None
prev_pwm_right = None
TEST_MODE = False

# ----------------------------
# Utility Functions
# ----------------------------
def clamp(val, min_val, max_val):
    """Limit val to [min_val, max_val] for hardware safety."""
    return max(min(val, max_val), min_val)

def normalize_rpm(rpm):
    """Normalize RPM for PID [0.0, 1.0]."""
    return (rpm - RPM_MIN) / (RPM_MAX - RPM_MIN)

def denormalize_pwm(norm):
    """Convert normalized PID output [0,1] to actual hardware PWM."""
    return int(PWM_MIN + norm * (PWM_MAX - PWM_MIN))

# ----------------------------
# Input Mapping: Turning Angle & Speed to Track Setpoints
# ----------------------------
def calculate_setpoints(norm_velocity, turning_angle):
    """
    Map velocity and steering to individual track setpoints.
    turning_angle: -90 (hard left) to +90 (hard right)
    """
    steering = clamp(turning_angle / 90.0, -1, 1)
    if steering <= 0:
        left_set = norm_velocity
        right_set = norm_velocity * (1 + steering)
    else:
        left_set = norm_velocity * (1 - steering)
        right_set = norm_velocity
    log("Input Map: V=%.2f, Steer=%.2f → L=%.2f, R=%.2f", norm_velocity, steering, left_set, right_set)
    return left_set, right_set

# ----------------------------
# Read Motor RPMs from Encoders
# ----------------------------
def read_rpms_pio():
    """
    Reads left and right output shaft RPMs via Pololu 37D encoders using Pico PIO.
    Assumes enc_right/enc_left are PIOQuadratureCounter instances (see dual_rpm_pio_test).
    """
    right_rpm, left_rpm = read_rpms(enc_right, enc_left, interval_sec=0.05)
    log("Encoder RPMs: Left=%.1f, Right=%.1f", left_rpm, right_rpm)
    return left_rpm, right_rpm

# ----------------------------
# PID Controller Setup (per motor track)
# ----------------------------
pid_left = PID(PID_KP, PID_KI, PID_KD, setpoint=0, sample_time=None,
               integral_limits=(PID_INTEGRAL_MIN, PID_INTEGRAL_MAX), output_limits=(PID_OUTPUT_MIN, PID_OUTPUT_MAX))
pid_right = PID(PID_KP, PID_KI, PID_KD, setpoint=0, sample_time=None,
                integral_limits=(PID_INTEGRAL_MIN, PID_INTEGRAL_MAX), output_limits=(PID_OUTPUT_MIN, PID_OUTPUT_MAX))

# ----------------------------
# Cross-Coupled PID Core Logic (handles turns, diff drive)
# ----------------------------
def cross_coupled_pid_control(signed_rpm_setpoint: int, turning_angle: float):
    """
    Core PID logic:
    - Converts navigation setpoint + steering to normalized left/right setpoints.
    - Reads actual RPMs.
    - Calculates PID output for each motor.
    - Optionally logs/flags oscillation in TEST_MODE.
    Returns (left_pwm, right_pwm, oscillation_detected, target_reached)
    """
    global prev_pwm_left, prev_pwm_right
    norm_v = normalize_rpm(signed_rpm_setpoint)
    log("Setpoint: RPM=%d, Angle=%.2f", signed_rpm_setpoint, turning_angle)
    left_set, right_set = calculate_setpoints(norm_v, turning_angle)

    measured_left, measured_right = read_rpms_pio()
    norm_measured_left = normalize_rpm(measured_left)
    norm_measured_right = normalize_rpm(measured_right)

    left_error = left_set - norm_measured_left
    right_error = right_set - norm_measured_right
    desired_diff = right_set - left_set
    actual_diff = norm_measured_right - norm_measured_left
    diff_error = desired_diff - actual_diff

    log("PID Errors: Left=%.2f, Right=%.2f, Diff=%.2f", left_error, right_error, diff_error)

    pid_left.setpoint = left_set
    pid_right.setpoint = right_set

    left_pid_out = pid_left(norm_measured_left)
    right_pid_out = pid_right(norm_measured_right)

    log("PID Out: Left=%.3f, Right=%.3f", left_pid_out, right_pid_out)

    left_pwm = int(clamp(denormalize_pwm(left_pid_out), PWM_MIN, PWM_MAX))
    right_pwm = int(clamp(denormalize_pwm(right_pid_out), PWM_MIN, PWM_MAX))

    # Oscillation detection (optional, for test/debug only)
    oscillation_detected = False
    target_reached = False
    if TEST_MODE:
        if prev_pwm_left is not None and (left_pwm < prev_pwm_left - OSCILLATION_THRESHOLD):
            oscillation_detected = True
            log("Oscillation detected! Left PWM dropped.")
        if prev_pwm_right is not None and (right_pwm < prev_pwm_right - OSCILLATION_THRESHOLD):
            oscillation_detected = True
            log("Oscillation detected! Right PWM dropped.")
        if (measured_left > (abs(signed_rpm_setpoint) - abs(signed_rpm_setpoint) * 0.05)
                and measured_right > (abs(signed_rpm_setpoint) - abs(signed_rpm_setpoint) * 0.05)):
            target_reached = True

    prev_pwm_left = left_pwm
    prev_pwm_right = right_pwm

    log("PWM Output: Left=%d, Right=%d", left_pwm, right_pwm)

    return left_pwm, right_pwm, oscillation_detected, target_reached

# ----------------------------
# Motor Driver Output & Emergency Brake
# ----------------------------
def set_motor_output(pwm_value, pwm_obj, dir_obj, dir_value: int, label=""):
    """
    Set Cytron MDD10A motor channel PWM and direction.
    - pwm_value: 0..65535
    - dir_value: 0=reverse, 1=forward (match your wiring/robot logic)
    """
    pwm = int(clamp(pwm_value, 0, 65535))
    pwm_obj.freq(PWM_FREQ)
    pwm_obj.duty_u16(pwm)
    dir_obj.value(dir_value)
    log("%s Motor: PWM=%d, DIR=%s", label, pwm, "FWD" if dir_value == 1 else "REV")

def stop_motors():
    """
    Stops both motors with Cytron MDD10A "hardware brake" (see truth table in datasheet).
    Both DIR and PWM high = fast brake (shorts both outputs).
    """
    global current_pwm_left, current_pwm_right
    if PWM_LEFT.duty_u16() == 0 and PWM_RIGHT.duty_u16() == 0:
        log("Motors already stopped.")
        return
    # Fast brake, Cytron truth table: both HIGH is BRAKE (or both LOW)
    PWM_LEFT.duty_u16(65535)
    DIR_LEFT.value(0)
    PWM_RIGHT.duty_u16(65535)
    DIR_RIGHT.value(0)
    current_pwm_left = 0
    current_pwm_right = 0
    log("Emergency brake applied (hardware). Motors stopped.")

def terminate_thread():
    """Stop control loop and ensure robot is safely halted."""
    global terminate
    stop_motors()
    log("Terminating robot control loop...")
    terminate = True

# ----------------------------
# Main Robot Drive Thread (for _thread on Pico)
# ----------------------------
def main_control_loop():
    """
    Main robot drive control thread:
    - Initializes hardware encoders on each run.
    - Handles navigation updates and PID-based drive control.
    - Applies hardware braking on stop.
    """
    start_time = time.time()

    global current_pwm_left, current_pwm_right, terminate, enc_right, enc_left
    enc_right = PIOQuadratureCounter(0, 10, 11)  # Pico PIO SM0, right motor encoder
    enc_left = PIOQuadratureCounter(1, 20, 21)   # Pico PIO SM1, left motor encoder

    terminate = False  # Allow thread to run

    while not terminate:
        navigation_params = driverController.get_navigation_params()
        log("Nav: stop=%s, angle=%.2f, rpm=%d",
            navigation_params.stop,
            navigation_params.target_angle,
            navigation_params.get_signed_target_rpm()
        )

        signed_rpm_setpoint = navigation_params.get_signed_target_rpm()
        turning_angle = navigation_params.target_angle
        if TEST_MODE:
            signed_rpm_setpoint = 330  # For simulated straight run
            turning_angle = 0.0

        left_pwm, right_pwm, oscillation_detected, target_reached = cross_coupled_pid_control(
            signed_rpm_setpoint, turning_angle)

        if TEST_MODE and oscillation_detected:
            log("Oscillation detected. Breaking control loop.")
            break

        if TEST_MODE and target_reached:
            log("Target RPM reached. Ending test mode loop.")
            break

        if navigation_params.stop:
            log("Stop requested in navigation parameters. Stopping loop.")
            break

        if current_pwm_left != left_pwm:
            set_motor_output(left_pwm, PWM_LEFT, DIR_LEFT, 0, "Left")
            current_pwm_left = left_pwm
        if current_pwm_right != right_pwm:
            set_motor_output(right_pwm, PWM_RIGHT, DIR_RIGHT, 0, "Right")
            current_pwm_right = right_pwm

        time.sleep(0.01)  # 10ms PID/control sample

    stop_motors()
    end_time = time.time()
    elapsed_seconds = end_time - start_time

    log("Control loop terminated.")
    log("Total control loop time: %.3f seconds", elapsed_seconds)

    # Hardware cleanup for PIO encoders (important for restart!)
    PWM_LEFT.duty_u16(0)
    PWM_RIGHT.duty_u16(0)
    enc_right.deinit()
    enc_left.deinit()
