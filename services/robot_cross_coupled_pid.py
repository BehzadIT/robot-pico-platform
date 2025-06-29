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
PWM_LEFT  = PWM(Pin(9))
DIR_LEFT  = Pin(8, Pin.OUT)

# Encoder pins are handled in your PIO-based RPM module.
# Import your dual_rpm_pio_test module or ensure the following functions exist:
# dual_rpm_pio_test.read_left_motor_rpm()  -> int
# dual_rpm_pio_test.read_right_motor_rpm() -> int

# ----------------------------
# PID Configuration Parameters
# ----------------------------
# These need to be tuned for your robot!
PID_KP = 1.2
PID_KI = 0.12
PID_KD = 0.01

K_DELTA = 0.5  # Cross-coupling strength (tune this for best turning response)

PID_OUTPUT_MIN = 0
PID_OUTPUT_MAX = 65535

PWM_FREQ = 20000  # 20kHz for smooth Cytron MDD10A operation

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
def calculate_setpoints(V, S):
    # Clamp steering to [-1, +1]
    S = clamp(S, -1, 1)
    if S <= 0:
        left_setpoint = V
        right_setpoint = V * (1 + S)
    else:
        left_setpoint = V * (1 - S)
        right_setpoint = V
    print("[Input Mapping] V={:.2f}, S={:.2f} → L_set={:.2f}, R_set={:.2f}".format(V, S, left_setpoint, right_setpoint))
    return left_setpoint, right_setpoint

# ----------------------------
# Step 2: RPM Measurement
# ----------------------------
def read_rpms_pio():
    right_rpm, left_rpm = read_rpms(enc_right, enc_left, interval_sec=0.05)
    print("[RPM] Left: {}, Right: {}".format(left_rpm, right_rpm))
    return left_rpm, right_rpm

# ----------------------------
# Step 3: PID Controllers Setup
# ----------------------------
# Initialize PID controllers (anti-windup, output clamping)
pid_left = PID(PID_KP, PID_KI, PID_KD, setpoint=0, sample_time=0.05, output_limits=(PID_OUTPUT_MIN, PID_OUTPUT_MAX))
pid_right = PID(PID_KP, PID_KI, PID_KD, setpoint=0, sample_time=0.05, output_limits=(PID_OUTPUT_MIN, PID_OUTPUT_MAX))

# ----------------------------
# Step 4: Cross-Coupled PID Logic
# ----------------------------
def cross_coupled_pid_control(V, S):
    # Get setpoints based on steering command
    left_set, right_set = calculate_setpoints(V, S)

    # Get actual measured speeds
    measured_left, measured_right = read_rpms_pio()

    # Compute errors
    left_error = left_set - measured_left
    right_error = right_set - measured_right
    desired_diff = right_set - left_set
    actual_diff = measured_right - measured_left
    diff_error = desired_diff - actual_diff

    print("[Errors] Left: {:.2f}, Right: {:.2f}, Diff: {:.2f}".format(left_error, right_error, diff_error))

    # Set new setpoints for the PIDs (not just once at init)
    pid_left.update_setpoint(left_set,50, 0)
    pid_right.update_setpoint(right_set,50, 0)

    # Compute PID outputs
    left_pid_out = pid_left(measured_left)
    right_pid_out = pid_right(measured_right)

    print("[PID Output] Left: {:.2f}, Right: {:.2f}".format(left_pid_out, right_pid_out))

    # Cross-coupling adjustment
    left_final = left_pid_out - (K_DELTA * diff_error)
    right_final = right_pid_out + (K_DELTA * diff_error)

    # Clamp to PWM range
    left_final = int(clamp(left_final, PID_OUTPUT_MIN, PID_OUTPUT_MAX))
    right_final = int(clamp(right_final, PID_OUTPUT_MIN, PID_OUTPUT_MAX))

    print("[Cross-Coupled Output] Left: {}, Right: {}".format(left_final, right_final))

    return left_final, right_final

# ----------------------------
# Step 5: Apply PWM & Direction
# ----------------------------
def set_motor_output(pwm_value, pwm_obj, dir_obj,dir_value: int, label=""):
    pwm = int(clamp(pwm_value, 0, 65535))
    pwm_obj.freq(PWM_FREQ)
    pwm_obj.duty_u16(pwm)
    dir_obj.value(dir_value)  # Set direction pin
    print("[{} Motor] PWM: {}, DIR: {}".format(label, pwm, "FWD" if dir_value ==1 else "REV"))

def angle_to_steering(angle):
    return angle / 90.0
# ----------------------------
# Main Control Loop
# ----------------------------
def main_control_loop():
    # Example input: V = 150 (RPM), S = 0.0 (straight)
    # Set as needed (RPM, or ticks/sec, etc.)
    # S = 0.8          # Steering: -1.0 to +1.0
    NUM_STEPS = 500  # Number of control steps to run
    # Main loop (adjust timing/sample_time as needed)
    # init_pwm = 20000
    # PWM_LEFT.duty_u16(init_pwm)  # Set initial PWM for left motor
    # PWM_RIGHT.duty_u16(init_pwm)  # Set initial PWM for right motor

    for step in range(NUM_STEPS):
        navigation_params = driverController.get_navigation_params()
        V = navigation_params.rpm
        S = angle_to_steering(navigation_params.angle)  # Convert angle to steering

        # TODO: Replace V and S with your own command input system if needed
        left_pwm, right_pwm = cross_coupled_pid_control(V, S)

        # Apply to hardware
        set_motor_output(right_pwm, PWM_RIGHT, DIR_RIGHT,0,"Right")
        set_motor_output(left_pwm, PWM_LEFT, DIR_LEFT,0,"Left")

        # Loop at PID sample rate for stable control
        time.sleep(pid_left.sample_time)

    print("Done. Stopping motors.")
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
