# robot_cross_coupled_pid.py
# Differential drive robot cross-coupled PID controller for Raspberry Pi Pico (MicroPython)
# Dependencies: simple_pid library (put simple_pid.py in the same directory or in your lib folder)

from machine import Pin, PWM
import time
from services.simple_pid import PID
from services.dual_rpm_pio_test import read_rpms, PIOQuadratureCounter
from services.robot_drive_controller import driverController

# ----------------------------
# Motor and Encoder Pin Setup
# ----------------------------
PWM_RIGHT = PWM(Pin(7))
DIR_RIGHT = Pin(6, Pin.OUT)
PWM_LEFT = PWM(Pin(9))
DIR_LEFT = Pin(8, Pin.OUT)

# Encoder pins are handled in your PIO-based RPM module.
# Import your dual_rpm_pio_test module or ensure the following functions exist:
# dual_rpm_pio_test.read_left_motor_rpm()  -> int
# dual_rpm_pio_test.read_right_motor_rpm() -> int

# ----------------------------
# Normalization/Denormalization Parameters
# ----------------------------
RPM_MIN = 0
RPM_MAX = 330
PWM_MIN = 5000
PWM_MAX = 65535


def norm_rpm(rpm):
    """Normalize RPM to 0-1"""
    return (rpm - RPM_MIN) / (RPM_MAX - RPM_MIN)


def denorm_pwm(norm):
    """Denormalize 0-1 to PWM range"""
    return int(PWM_MIN + norm * (PWM_MAX - PWM_MIN))


# ----------------------------
# PID Configuration Parameters
# ----------------------------
# These need to be tuned for your robot!
PID_KP = 0.485  # Tune for normalized input!
PID_KI = 0.90
PID_KD = 0

K_DELTA = 0.5  # Cross-coupling strength (tune this for best turning response)

PID_OUTPUT_MIN = 0.0
PID_OUTPUT_MAX = 1.0

PID_INTEGRAL_MIN = 0
PID_INTEGRAL_MAX = 1  # Integral windup limit (tune this for your robot)

PWM_FREQ = 20000  # 20kHz for smooth Cytron MDD10A operation

OSCILLATION_THRESHOLD = 10000  # Threshold for detecting oscillation in PWM

enc_right = PIOQuadratureCounter(0, 10, 11)
enc_left = PIOQuadratureCounter(1, 20, 21)


# ----------------------------
# Helper: Clamp Output
# ----------------------------
def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)


# ----------------------------
# Step 1: Input Mapping
# ----------------------------
def calculate_setpoints(norm_V, turning_angle: float):
    # Convert angle to steering and Clamp steering to [-1, +1]
    steering_angle = clamp(turning_angle / 90.0, -1, 1)
    if steering_angle <= 0:
        left_setpoint = norm_V
        right_setpoint = norm_V * (1 + steering_angle)
    else:
        left_setpoint = norm_V * (1 - steering_angle)
        right_setpoint = norm_V
    print(
        "[Input Mapping] V={:.2f}, S={:.2f} → L_set={:.2f}, R_set={:.2f}".format(norm_V, steering_angle, left_setpoint,
                                                                                 right_setpoint))
    return left_setpoint, right_setpoint


# ----------------------------
# Step 2: RPM Measurement
# ----------------------------
def read_rpms_pio():
    right_rpm, left_rpm = read_rpms(enc_right, enc_left, interval_sec=0.05)
    print("[RPM] Left: {}, Right: {}".format(left_rpm, right_rpm))
    return left_rpm, right_rpm


# ----------------------------
# Step 3: PID Controllers Setup (Normalized)
# ----------------------------
# Initialize PID controllers (anti-windup, output clamping)
pid_left = PID(PID_KP, PID_KI, PID_KD, setpoint=0, sample_time=None,
               integral_limits=(PID_INTEGRAL_MIN, PID_INTEGRAL_MAX), output_limits=(PID_OUTPUT_MIN, PID_OUTPUT_MAX))
pid_right = PID(PID_KP, PID_KI, PID_KD, setpoint=0, sample_time=None,
                integral_limits=(PID_INTEGRAL_MIN, PID_INTEGRAL_MAX), output_limits=(PID_OUTPUT_MIN, PID_OUTPUT_MAX))

prev_pwm_left = None
prev_pwm_right = None


# ----------------------------
# Step 4: Cross-Coupled PID Logic
# ----------------------------
def cross_coupled_pid_control(signed_rpm_setpoint: int, turing_angle: float):
    global prev_pwm_left, prev_pwm_right
    # Normalize setpoint before passing to PID
    norm_V = norm_rpm(signed_rpm_setpoint)
    print("signed_rpm_setpoint: {}, turing_angle: {}".format(signed_rpm_setpoint, turing_angle))
    # Get setpoints based on steering command (normalized)
    left_set, right_set = calculate_setpoints(norm_V, turing_angle)

    # Get actual measured speeds (raw RPM)
    measured_left, measured_right = read_rpms_pio()
    norm_measured_left = norm_rpm(measured_left)
    norm_measured_right = norm_rpm(measured_right)

    # Compute errors (optional, for logging)
    left_error = left_set - norm_measured_left
    right_error = right_set - norm_measured_right

    desired_diff = right_set - left_set
    actual_diff = norm_measured_right - norm_measured_left
    diff_error = desired_diff - actual_diff

    print("[Errors] Left: {:.2f}, Right: {:.2f}, Diff: {:.2f}".format(left_error, right_error, diff_error))

    # Set new setpoints for the PIDs (normalized)
    pid_left.setpoint = left_set
    pid_right.setpoint = right_set

    # Compute PID outputs (normalized)
    left_pid_out = pid_left(norm_measured_left)
    right_pid_out = pid_right(norm_measured_right)

    print("[PID Output] Left: {:.3f}, Right: {:.3f}".format(left_pid_out, right_pid_out))

    # Cross-coupling adjustment (normalized) (uncomment if needed)
    # left_pid_out -= K_DELTA * diff_error
    # right_pid_out += K_DELTA * diff_error

    # Clamp to normalized range, then denormalize to PWM
    left_final_pwm = int(clamp(denorm_pwm(left_pid_out), PWM_MIN, PWM_MAX))
    right_final_pwm = int(clamp(denorm_pwm(right_pid_out), PWM_MIN, PWM_MAX))
    # Oscillation detection, since we start with 0 rpm and target is 330 RPM, any decrease in PWM is considered oscillation
    OSCILLATION_DETECTED = False
    if prev_pwm_left is not None and (left_final_pwm < prev_pwm_left - OSCILLATION_THRESHOLD):
        OSCILLATION_DETECTED = True
        print("OSCILLATION DETECTED: PID new pwm: {} < prev pwm: {}".format(left_final_pwm, prev_pwm_left))
    if prev_pwm_right is not None and (right_final_pwm < prev_pwm_right - OSCILLATION_THRESHOLD):
        OSCILLATION_DETECTED = True
        print("OSCILLATION DETECTED: PID new pwm: {} < prev pwm: {}".format(right_final_pwm, prev_pwm_right))

    prev_pwm_left = left_final_pwm
    prev_pwm_right = right_final_pwm

    print("[Cross-Coupled Output] Left: {}, Right: {}".format(left_final_pwm, right_final_pwm))

    TARGET_REACHED = False
    if (measured_left > (abs(signed_rpm_setpoint) - abs(signed_rpm_setpoint) * 0.05)
            and measured_right > (abs(signed_rpm_setpoint) - abs(signed_rpm_setpoint) * 0.05)):
        TARGET_REACHED = True


    return left_final_pwm, right_final_pwm, OSCILLATION_DETECTED, TARGET_REACHED


# ----------------------------
# Step 5: Apply PWM & Direction
# ----------------------------
def set_motor_output(pwm_value, pwm_obj, dir_obj, dir_value: int, label=""):
    pwm = int(clamp(pwm_value, 0, 65535))
    pwm_obj.freq(PWM_FREQ)
    pwm_obj.duty_u16(pwm)
    dir_obj.value(dir_value)  # Set direction pin
    print("[{} Motor] PWM: {}, DIR: {}".format(label, pwm, "FWD" if dir_value == 1 else "REV"))


# ----------------------------
# Main Control Loop
# ----------------------------
def main_control_loop():
    # Example input: V = 150 (RPM), S = 0.0 (straight)
    # Set as needed (RPM, or ticks/sec, etc.)
    # S = 0.8          # Steering: -1.0 to +1.0
    NUM_STEPS = 100  # Number of control steps to run
    # Main loop (adjust timing/sample_time as needed)
    # init_pwm = 20000
    # PWM_LEFT.duty_u16(init_pwm)  # Set initial PWM for left motor
    # PWM_RIGHT.duty_u16(init_pwm)  # Set initial PWM for right motor
    start_time = time.time()

    for step in range(NUM_STEPS):
        navigation_params = driverController.get_navigation_params()
        signed_rpm_setpoint = navigation_params.get_signed_target_rpm()
        turing_angle = navigation_params.target_angle

        left_pwm, right_pwm, oscillation_detected, target_reached = cross_coupled_pid_control(signed_rpm_setpoint,
                                                                                              turing_angle)
        if oscillation_detected:
            break

        if target_reached:
            print("Target RPM reached. Stopping control loop.")
            break

        # Apply to hardware
        set_motor_output(right_pwm, PWM_RIGHT, DIR_RIGHT, 0, "Right")
        set_motor_output(left_pwm, PWM_LEFT, DIR_LEFT, 0, "Left")

        # Loop at PID sample rate for stable control
        time.sleep(0.01)  # 10ms sample time

    end_time = time.time()
    elapsed_seconds = end_time - start_time

    print("Done. Stopping motors.")
    print("Total loop time: {:.3f} seconds".format(elapsed_seconds))

    PWM_LEFT.duty_u16(0)
    PWM_RIGHT.duty_u16(0)
    enc_right.deinit()
    enc_left.deinit()

# ----------------------------
# Run the main loop if file executed directly
# ----------------------------
# if __name__ == "__main__":
#     try:
#         print("Starting Cross-Coupled PID Differential Drive Control...")
#         main_control_loop()
#     except KeyboardInterrupt:
#         print("Control loop stopped by user.")
#         # Optionally stop motors
#         PWM_RIGHT.duty_u16(0)
#         PWM_LEFT.duty_u16(0)
