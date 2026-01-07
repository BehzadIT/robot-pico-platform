# robot_cross_coupled_pid.py
# Differential drive robot cross-coupled PID controller for Raspberry Pi Pico (MicroPython)
# Hardware: Pololu 37D motors & encoders, Cytron MDD10A motor driver
# Dependencies: simple_pid.py in same directory or lib

from machine import Pin, PWM
import time
from log import *
from services.simple_pid import PID
from services.dual_rpm_pio_test import read_rpms, PIOQuadratureCounter


class RobotPID:
    # ----------------------------
    # Hardware Pin Setup - Pico to Cytron MDD10A, Pololu 37D
    # ----------------------------
    # Pico GPIO Pin mapping (adjust to your wiring as needed)
    PWM_RIGHT = PWM(Pin(7))  # Right motor PWM (Cytron MDD10A PWM2 pin)
    DIR_RIGHT = Pin(6, Pin.OUT)  # Right motor DIR (Cytron MDD10A DIR2 pin)
    PWM_LEFT = PWM(Pin(9))  # Left motor PWM (Cytron MDD10A PWM1 pin)
    DIR_LEFT = Pin(8, Pin.OUT)  # Left motor DIR (Cytron MDD10A DIR1 pin)
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
    terminate_flag = False

    # ----------------------------
    # PID Constants (Tune for your robot!)
    # ----------------------------
    PID_KP = 0.489
    PID_KI = 0.92
    PID_KD = 0

    K_DELTA = 0.5  # Cross-coupling strength (affects turn sharpness)
    PID_OUTPUT_MIN = 0.0    # Minimum PID output (0% power)
    PID_OUTPUT_MAX = 1.0    # Maximum PID output (100% power)
    PID_INTEGRAL_MIN = 0.0  # Minimum integral term
    PID_INTEGRAL_MAX = 1.0  # Maximum integral term
    PWM_FREQ = 20000  # 20kHz = silent and efficient on Cytron

    OSCILLATION_THRESHOLD = 10000  # Used in TEST_MODE for stability testing

    current_pwm_right = 0
    current_pwm_left = 0
    prev_pwm_left = None
    prev_pwm_right = None
    test_mode = False

    def __init__(self):
        self.terminate_flag = True
        self.enc_right = None
        self.enc_left = None
        self.current_pwm_right = 0
        self.current_pwm_left = 0
        self.prev_pwm_left = None
        self.prev_pwm_right = None

        # ----------------------------
        # PID Controller Setup (per motor track)
        # ----------------------------
        self.pid_left = PID(self.PID_KP, self.PID_KI, self.PID_KD, setpoint=0, sample_time=None,
                            integral_limits=(self.PID_INTEGRAL_MIN, self.PID_INTEGRAL_MAX),
                            output_limits=(self.PID_OUTPUT_MIN, self.PID_OUTPUT_MAX))
        self.pid_right = PID(self.PID_KP, self.PID_KI, self.PID_KD, setpoint=0, sample_time=None,
                             integral_limits=(self.PID_INTEGRAL_MIN, self.PID_INTEGRAL_MAX),
                             output_limits=(self.PID_OUTPUT_MIN, self.PID_OUTPUT_MAX))

    @staticmethod
    def log(msg, *args):
        """Uniform log output for PID and hardware events."""
        try:
            logd("[PID] " + (msg % args if args else msg))
        except Exception as e:
            logw(f"[PID] Error logging message: {e}")
            logd(f"[PID] {msg}")

    @staticmethod
    def clamp(val, min_val, max_val):
        """Limit val to [min_val, max_val] for hardware safety."""
        return max(min(val, max_val), min_val)

    def start(self, navigation_controller):
        """Starts the PID control thread; navigation_controller must have get_navigation_params()."""
        import _thread
        if not self.terminate_flag:
            self.log("PID thread already running.")
            return
        self.terminate_flag = False
        self.log("Starting cross-coupled PID control thread.")
        _thread.start_new_thread(self.main_control_loop, (navigation_controller,))

    def normalize_rpm(self, rpm):
        """Normalize RPM for PID [-1.0, 1.0] where negative is reverse."""
        return rpm / self.RPM_MAX  # This will give us a range of [-1.0, 1.0]

    def denormalize_pwm(self, norm):
        """Convert normalized PID output to actual hardware PWM, preserving sign."""
        pwm_range = self.PWM_MAX - self.PWM_MIN
        if norm < 0:
            return -int(self.PWM_MIN + abs(norm) * pwm_range)
        return int(self.PWM_MIN + norm * pwm_range)

    def calculate_setpoints(self, norm_velocity, turning_angle):
        """
        Map velocity and steering to individual track setpoints.
        turning_angle: -90 (hard left) to +90 (hard right)
        """
        steering = self.clamp(turning_angle / 90.0, -1, 1)
        if steering <= 0:
            left_set = norm_velocity
            right_set = norm_velocity * (1 + steering)
        else:
            left_set = norm_velocity * (1 - steering)
            right_set = norm_velocity
        self.log("Input Map: V=%.2f, Steer=%.2f → L=%.2f, R=%.2f", norm_velocity, steering, left_set, right_set)
        return left_set, right_set

    def read_rpms_pio(self):
        """
        Reads left and right output shaft RPMs via Pololu 37D encoders using Pico PIO.
        Assumes enc_right/enc_left are PIOQuadratureCounter instances (see dual_rpm_pio_test).
        """
        right_rpm, left_rpm = read_rpms(self.enc_right, self.enc_left, interval_sec=0.05)
        self.log("Encoder RPMs: Left=%.1f, Right=%.1f", left_rpm, right_rpm)
        return left_rpm, right_rpm

    def cross_coupled_pid_control(self, signed_rpm_setpoint: int, turning_angle: float):
        """
        Run one iteration of the cross-coupled PID control.
        - signed_rpm_setpoint: desired RPM (positive for forward, negative for reverse)
        - turning_angle: desired turning angle (in degrees)
        Returns (left_pwm, right_pwm, oscillation_detected, target_reached)
        """
        # Determine direction and use absolute RPM for calculations
        direction = -1 if signed_rpm_setpoint < 0 else 1
        abs_rpm = abs(signed_rpm_setpoint)

        # Log input parameters
        self.log("\n[PID] New Control Cycle - RPM: %d (%s), Angle: %.2f",
                 abs_rpm, "REVERSE" if direction < 0 else "FORWARD", turning_angle)

        # Normalize RPM (0.0 to 1.0)
        norm_velocity = self.normalize_rpm(abs_rpm)
        self.log("  Normalized Velocity: %.2f (RPM: %d)", norm_velocity, abs_rpm)

        # Calculate setpoints (0.0 to 1.0)
        left_set, right_set = self.calculate_setpoints(norm_velocity, turning_angle)
        self.log("  Setpoints - Left: %.3f, Right: %.3f", left_set, right_set)

        # Read actual RPMs and normalize (take absolute value for PID)
        measured_left, measured_right = self.read_rpms_pio()
        abs_measured_left = abs(measured_left)
        abs_measured_right = abs(measured_right)
        norm_measured_left = self.normalize_rpm(abs_measured_left)
        norm_measured_right = self.normalize_rpm(abs_measured_right)

        self.log("  Measured RPMs - Left: %.1f (abs: %.1f, norm: %.3f), Right: %.1f (abs: %.1f, norm: %.3f)",
                 measured_left, abs_measured_left, norm_measured_left,
                 measured_right, abs_measured_right, norm_measured_right)

        # Calculate errors (use absolute values)
        left_error = left_set - norm_measured_left
        right_error = right_set - norm_measured_right
        self.log("  Errors - Left: %.3f, Right: %.3f", left_error, right_error)

        # Update PID setpoints
        self.pid_left.setpoint = left_set
        self.pid_right.setpoint = right_set

        # Get PID outputs (0.0 to 1.0)
        left_pid_out = self.pid_left(norm_measured_left)
        right_pid_out = self.pid_right(norm_measured_right)
        self.log("  PID Outputs - Left: %.3f, Right: %.3f", left_pid_out, right_pid_out)

        # Convert to PWM and apply direction
        left_pwm = int(self.denormalize_pwm(left_pid_out)) * direction
        right_pwm = int(self.denormalize_pwm(right_pid_out)) * direction

        # Clamp to valid PWM range
        left_pwm = int(self.clamp(left_pwm, -65535, 65535))
        right_pwm = int(self.clamp(right_pwm, -65535, 65535))

        self.log("  Final PWM - Left: %d, Right: %d", left_pwm, right_pwm)

        # Rest of the method remains the same...
        oscillation_detected = False
        target_reached = False

        if self.test_mode:
            if self.prev_pwm_left is not None and (
                    abs(left_pwm) < abs(self.prev_pwm_left) - self.OSCILLATION_THRESHOLD):
                oscillation_detected = True
                self.log("  Oscillation detected! Left PWM dropped.")
            if self.prev_pwm_right is not None and (
                    abs(right_pwm) < abs(self.prev_pwm_right) - self.OSCILLATION_THRESHOLD):
                oscillation_detected = True
                self.log("  Oscillation detected! Right PWM dropped.")
            if (abs_measured_left > (abs_rpm - abs_rpm * 0.05)
                    and abs_measured_right > (abs_rpm - abs_rpm * 0.05)):
                target_reached = True

        self.prev_pwm_left = left_pwm
        self.prev_pwm_right = right_pwm

        return left_pwm, right_pwm, oscillation_detected, target_reached

    def set_motor_output(self, pwm_value, pwm_obj, dir_obj, dir_value: int, label=""):
        """
        Set Cytron MDD10A motor channel PWM and direction.
        - pwm_value: signed value where sign indicates direction
        - dir_value: 0=reverse, 1=forward (match your wiring/robot logic)
        """
        # Determine direction based on sign of pwm_value
        if pwm_value < 0:
            dir_value = 1  # Reverse direction
            pwm_value = abs(pwm_value)  # PWM is always positive
        else:
            dir_value = 0  # Forward direction

        # Apply deadband and ensure we're within PWM limits
        if abs(pwm_value) < self.PWM_MIN:
            pwm_value = 0
            
        pwm = int(self.clamp(pwm_value, 0, 65535))
        pwm_obj.freq(self.PWM_FREQ)
        pwm_obj.duty_u16(pwm)
        dir_obj.value(dir_value)
        self.log("%s Motor: PWM=%d, DIR=%s", label, pwm, "FWD" if dir_value == 0 else "REV")

    def stop_motors(self):
        """
        Stops both motors with Cytron MDD10A "hardware brake" (see truth table in datasheet).
        Both DIR and PWM high = fast brake (shorts both outputs).
        """
        if self.PWM_LEFT.duty_u16() == 0 and self.PWM_RIGHT.duty_u16() == 0:
            self.log("Motors already stopped.")
            return
        self.PWM_LEFT.duty_u16(0)
        self.PWM_RIGHT.duty_u16(0)
        self.current_pwm_left = 0
        self.current_pwm_right = 0
        self.log("Emergency brake applied (hardware). Motors stopped.")

    def terminate_thread(self):
        """Stop control loop and ensure robot is safely halted."""
        self.stop_motors()
        self.log("Terminating robot control loop...")
        self.terminate_flag = True

    def main_control_loop(self, driver_controller):
        """
        Main robot drive control thread:
        - Initializes hardware encoders on each run.
        - Handles navigation updates and PID-based drive control.
        - Applies hardware braking on stop.
        """
        start_time = time.time()

        self.enc_right = PIOQuadratureCounter(0, 10, 11)  # Pico PIO SM0, right motor encoder
        self.enc_left = PIOQuadratureCounter(1, 20, 21)  # Pico PIO SM1, left motor encoder

        self.terminate_flag = False  # Allow thread to run

        while not self.terminate_flag:
            navigation_params = driver_controller.get_navigation_params()
            self.log("Nav: angle=%.2f, rpm=%d",
                     navigation_params.target_angle,
                     navigation_params.get_signed_target_rpm())

            signed_rpm_setpoint = navigation_params.get_signed_target_rpm()
            turning_angle = navigation_params.target_angle
            if self.test_mode:
                signed_rpm_setpoint = 330  # For simulated straight run
                turning_angle = 0.0

            left_pwm, right_pwm, oscillation_detected, target_reached = self.cross_coupled_pid_control(
                signed_rpm_setpoint, turning_angle)

            if self.test_mode and oscillation_detected:
                self.log("Oscillation detected. Breaking control loop.")
                break

            if self.test_mode and target_reached:
                self.log("Target RPM reached. Ending test mode loop.")
                break

            if self.current_pwm_left != left_pwm:
                self.set_motor_output(left_pwm, self.PWM_LEFT, self.DIR_LEFT, 0, "Left")
                self.current_pwm_left = left_pwm
            if self.current_pwm_right != right_pwm:
                self.set_motor_output(right_pwm, self.PWM_RIGHT, self.DIR_RIGHT, 0, "Right")
                self.current_pwm_right = right_pwm

            time.sleep(0.01)  # 10ms PID/control sample

        self.stop_motors()
        end_time = time.time()
        elapsed_seconds = end_time - start_time

        self.log("Control loop terminated.")
        self.log("Total control loop time: %.3f seconds", elapsed_seconds)

        # Hardware cleanup for PIO encoders (important for restart!)
        # self.PWM_LEFT.duty_u16(0)
        # self.PWM_RIGHT.duty_u16(0)
        self.enc_right.deinit()
        self.enc_left.deinit()


robot_pid = RobotPID()
