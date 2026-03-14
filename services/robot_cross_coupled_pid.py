# Differential drive robot control for Raspberry Pi Pico / Pico 2.
#
# Phase 1 design:
# - one persistent worker thread owns drivetrain updates
# - one persistent encoder subsystem owns both PIO state machines
# - hardware faults are latched and future drive commands are rejected
#
# Phase 2 design:
# - the worker runs on a fixed 20 ms cadence
# - encoder velocity comes from non-blocking delta-count samples
# - PID state is reset deliberately on start/stop/fault/reversal

import _thread
import time
from machine import Pin, PWM

from log import logd, loge, logi, logw, telemetry
from services.drivetrain_constants import ControlTiming, MotorLimits, MotorPins, SteeringLimits
from services.encoder_subsystem import drivetrain_encoders
from services.simple_pid import PID


DRIVETRAIN_TAG = "drivetrain"


class RobotPID:
    """Long-lived drivetrain controller for the two tracked motors.

    This class owns the control-loop lifecycle, motor outputs, and PID state.
    The encoder hardware itself is owned by `encoder_subsystem.py`; this class
    consumes already-owned measurements and turns them into PWM updates.
    """
    PWM_RIGHT = PWM(Pin(MotorPins.RIGHT_PWM))
    DIR_RIGHT = Pin(MotorPins.RIGHT_DIR, Pin.OUT)
    PWM_LEFT = PWM(Pin(MotorPins.LEFT_PWM))
    DIR_LEFT = Pin(MotorPins.LEFT_DIR, Pin.OUT)

    RPM_MAX = MotorLimits.RPM_MAX
    PWM_MIN = MotorLimits.PWM_MIN
    PWM_MAX = MotorLimits.PWM_MAX
    PWM_FREQ = MotorLimits.PWM_FREQ

    PID_KP = 0.489
    PID_KI = 0.92
    PID_KD = 0
    PID_OUTPUT_MIN = 0.0
    PID_OUTPUT_MAX = 1.0
    PID_INTEGRAL_MIN = 0.0
    PID_INTEGRAL_MAX = 1.0
    OSCILLATION_THRESHOLD = 10000
    CONTROL_PERIOD_US = ControlTiming.CONTROL_PERIOD_US
    SUMMARY_PERIOD_MS = ControlTiming.SUMMARY_PERIOD_MS

    STATE_IDLE = "idle"
    STATE_STARTING = "starting"
    STATE_RUNNING = "running"
    STATE_STOPPING = "stopping"
    STATE_RECOVERING = "recovering"
    STATE_FAULTED = "faulted"

    def __init__(self):
        # One lock protects state/fault/worker ownership. The hot control path
        # avoids holding it for long so motor updates are not serialized behind
        # unrelated status reads.
        self._lock = _thread.allocate_lock()
        self._worker_started = False
        self._driver_controller = None
        self._state = self.STATE_IDLE
        self._fault = None
        self._last_fault = None

        self.current_pwm_right = 0
        self.current_pwm_left = 0
        self.prev_pwm_left = None
        self.prev_pwm_right = None
        self.test_mode = False

        self.pid_left = PID(
            self.PID_KP,
            self.PID_KI,
            self.PID_KD,
            setpoint=0,
            sample_time=None,
            scale="us",
            integral_limits=(self.PID_INTEGRAL_MIN, self.PID_INTEGRAL_MAX),
            output_limits=(self.PID_OUTPUT_MIN, self.PID_OUTPUT_MAX),
        )
        self.pid_right = PID(
            self.PID_KP,
            self.PID_KI,
            self.PID_KD,
            setpoint=0,
            sample_time=None,
            scale="us",
            integral_limits=(self.PID_INTEGRAL_MIN, self.PID_INTEGRAL_MAX),
            output_limits=(self.PID_OUTPUT_MIN, self.PID_OUTPUT_MAX),
        )

        self.PWM_LEFT.freq(self.PWM_FREQ)
        self.PWM_RIGHT.freq(self.PWM_FREQ)
        # Fixed-rate scheduling state. `_next_tick_us` is the scheduled deadline
        # for the next control update, not simply "last time we slept".
        self._next_tick_us = None
        self._last_tick_us = None
        # A reversal is handled as a short controlled stop so the controller
        # does not carry integral state directly through a sign flip.
        self._hold_zero_until_us = None
        self._last_target_sign = 0
        self._loop_overruns = 0
        self._last_summary_ms = time.ticks_ms()
        self._last_loop_measurement = None
        self._last_loop_target_rpm = 0
        self._last_loop_turning_angle = 0
        self._last_loop_dt_us = 0

    @staticmethod
    def _sign(value):
        if value > 0:
            return 1
        if value < 0:
            return -1
        return 0

    def _log_debug(self, msg, *args):
        logd(msg % args if args else msg, DRIVETRAIN_TAG)

    def _log_info(self, msg, *args):
        logi(msg % args if args else msg, DRIVETRAIN_TAG)

    def _log_warn(self, msg, *args):
        logw(msg % args if args else msg, DRIVETRAIN_TAG)

    def _log_error(self, msg, *args):
        loge(msg % args if args else msg, DRIVETRAIN_TAG)

    @staticmethod
    def clamp(val, min_val, max_val):
        return max(min(val, max_val), min_val)

    def _set_state_locked(self, new_state, reason=""):
        if self._state == new_state:
            return
        old_state = self._state
        self._state = new_state
        self._log_info("State transition: %s -> %s (%s)", old_state, new_state, reason or "no_reason")
        telemetry("drivetrain_state", old_state=old_state, new_state=new_state, reason=reason or "no_reason")

    def state(self):
        with self._lock:
            return self._state

    def is_running(self):
        return self.state() in (self.STATE_STARTING, self.STATE_RUNNING)

    def is_faulted(self):
        return self.state() == self.STATE_FAULTED

    def fault_info(self):
        with self._lock:
            return None if self._fault is None else dict(self._fault)

    def last_fault_info(self):
        with self._lock:
            return None if self._last_fault is None else dict(self._last_fault)

    def _clear_active_fault(self, reason):
        with self._lock:
            if self._fault is None:
                return
            cleared_fault = dict(self._fault)
            self._fault = None
        self._log_info("Clearing active drivetrain fault (%s)", reason)
        telemetry("drivetrain_fault_cleared", reason=reason, fault=cleared_fault)

    def _latch_fault(self, code, exc, extra=None):
        """Latch a drivetrain fault and force the controller into a safe stop.

        Faults are sticky by design. If encoder data or output control becomes
        untrustworthy, the robot should stop and reject future drive commands
        until a deliberate recovery path is added.
        """
        with self._lock:
            if self._fault is None:
                self._fault = {
                    "code": code,
                    "type": exc.__class__.__name__,
                    "message": str(exc),
                }
                if extra:
                    self._fault.update(extra)
                self._last_fault = dict(self._fault)
            self._set_state_locked(self.STATE_FAULTED, reason=code)

        self.reset_pid_state(code)
        self.stop_motors()
        self._log_error(
            "Latched drivetrain fault code=%s type=%s msg=%s",
            self._fault["code"],
            self._fault["type"],
            self._fault["message"],
        )
        telemetry("drivetrain_fault", **self._fault)

    def recover(self):
        """Recover the drivetrain only from a safe stopped state.

        Recovery is explicit and manual by design. It is rejected while the
        drivetrain is starting, running, or stopping so hardware reinit never
        races live motion.
        """
        with self._lock:
            if self._state not in (self.STATE_IDLE, self.STATE_FAULTED):
                detail = "recovery requires idle or faulted state"
                self._log_warn("Rejecting recovery while state=%s", self._state)
                telemetry("drivetrain_recover_rejected", state=self._state, reason="unsafe_state")
                return {"ok": False, "code": "rc", "detail": detail}
            self._set_state_locked(self.STATE_RECOVERING, reason="recover_requested")

        self.stop_motors()
        self.reset_pid_state("recover")
        self._next_tick_us = None
        self._last_tick_us = None
        self._last_loop_measurement = None
        self._clear_active_fault("recover_requested")

        try:
            drivetrain_encoders.recover()
            with self._lock:
                self._set_state_locked(self.STATE_IDLE, reason="recover_succeeded")
            self._log_info("Drivetrain recovery completed")
            telemetry("drivetrain_recovered", last_fault=self._last_fault)
            return {"ok": True, "code": "r", "detail": "recovered"}
        except Exception as exc:
            self._latch_fault("recovery_failed", exc, extra=drivetrain_encoders.fault_info())
            return {"ok": False, "code": "rf", "detail": str(exc)}

    def start(self, navigation_controller):
        """Ensure the persistent worker exists and request the running state.

        Recovery is exclusive: while `recovering`, no new drive start may
        interrupt encoder teardown/reinit or overwrite the pending state
        transition back to `idle`.
        """
        should_start_worker = False
        with self._lock:
            if self._state == self.STATE_FAULTED:
                self._log_warn("Ignoring start request while faulted")
                return {"ok": False, "code": "hf", "detail": "drivetrain faulted"}
            if self._state == self.STATE_RECOVERING:
                self._log_warn("Ignoring start request while recovery is in progress")
                return {"ok": False, "code": "ri", "detail": "recovery in progress"}

            self._driver_controller = navigation_controller

            if not self._worker_started:
                self._worker_started = True
                should_start_worker = True

            if self._state != self.STATE_RUNNING:
                self._set_state_locked(self.STATE_STARTING, reason="drive_requested")

        if should_start_worker:
            try:
                _thread.start_new_thread(self._worker_loop, ())
            except Exception as exc:
                with self._lock:
                    self._worker_started = False
                self._latch_fault("worker_start_failed", exc)
                return {"ok": False, "code": "hf", "detail": "worker start failed"}

        return {"ok": True}

    def terminate_thread(self):
        """Request a controlled stop.

        The historic name is kept for compatibility with existing callers. It
        no longer destroys the worker thread; it only moves the controller into
        `stopping` so the persistent worker can cleanly transition to `idle`.
        """
        with self._lock:
            if self._state == self.STATE_FAULTED:
                return
            self._set_state_locked(self.STATE_STOPPING, reason="stop_requested")
        self.stop_motors()

    def reset_pid_state(self, reason):
        """Reset all controller state that should not survive a motion segment.

        Non-obvious rule: start/stop/fault/reversal are treated as hard control
        boundaries. The integrator and last-input state must be cleared here so
        the next segment does not inherit stale control effort.
        """
        self.pid_left.reset()
        self.pid_right.reset()
        self.prev_pwm_left = None
        self.prev_pwm_right = None
        self._last_target_sign = 0
        self._hold_zero_until_us = None
        self._log_info("PID state reset (%s)", reason)
        telemetry("drivetrain_pid_reset", reason=reason)

    def normalize_rpm(self, rpm):
        return rpm / self.RPM_MAX

    def denormalize_pwm(self, norm):
        pwm_range = self.PWM_MAX - self.PWM_MIN
        if norm < 0:
            return -int(self.PWM_MIN + abs(norm) * pwm_range)
        return int(self.PWM_MIN + norm * pwm_range)

    def calculate_setpoints(self, norm_velocity, turning_angle):
        """Map one signed speed command into left/right normalized setpoints.

        The selected steering policy is inner-track slowdown:
        - `turning_angle == 0` keeps both tracks equal
        - positive angle steers right by slowing the right track
        - negative angle steers left by slowing the left track
        - the slower side approaches zero at +/-90 degrees
        """
        steering = self.clamp(
            turning_angle / float(SteeringLimits.ANGLE_MAX),
            -1,
            1,
        )
        if steering >= 0:
            left_set = norm_velocity
            right_set = norm_velocity * (1 - steering)
        else:
            left_set = norm_velocity * (1 + steering)
            right_set = norm_velocity
        self._log_debug(
            "Input map: velocity=%.2f steer=%.2f -> left=%.2f right=%.2f",
            norm_velocity,
            steering,
            left_set,
            right_set,
        )
        return left_set, right_set

    def cross_coupled_pid_control(self, signed_rpm_setpoint, turning_angle, measurement, dt_us):
        """Run one control update using the current encoder-derived velocity.

        `measurement` is expected to come from the persistent encoder
        subsystem's non-blocking delta-count estimator. `dt_us` is supplied
        explicitly so PID timing follows the real loop cadence instead of
        whatever wall-clock delay happened between calls.
        """
        direction = -1 if signed_rpm_setpoint < 0 else 1
        abs_rpm = abs(signed_rpm_setpoint)

        norm_velocity = self.normalize_rpm(abs_rpm)
        left_set, right_set = self.calculate_setpoints(norm_velocity, turning_angle)

        measured_left = measurement["left_rpm"]
        measured_right = measurement["right_rpm"]
        abs_measured_left = abs(measured_left)
        abs_measured_right = abs(measured_right)
        norm_measured_left = self.normalize_rpm(abs_measured_left)
        norm_measured_right = self.normalize_rpm(abs_measured_right)

        self.pid_left.setpoint = left_set
        self.pid_right.setpoint = right_set

        left_pid_out = self.pid_left(norm_measured_left, dt=dt_us)
        right_pid_out = self.pid_right(norm_measured_right, dt=dt_us)

        left_pwm = int(self.denormalize_pwm(left_pid_out)) * direction
        right_pwm = int(self.denormalize_pwm(right_pid_out)) * direction
        left_pwm = int(self.clamp(left_pwm, -65535, 65535))
        right_pwm = int(self.clamp(right_pwm, -65535, 65535))

        oscillation_detected = False
        target_reached = False
        if self.test_mode:
            if self.prev_pwm_left is not None and abs(left_pwm) < abs(self.prev_pwm_left) - self.OSCILLATION_THRESHOLD:
                oscillation_detected = True
            if self.prev_pwm_right is not None and abs(right_pwm) < abs(self.prev_pwm_right) - self.OSCILLATION_THRESHOLD:
                oscillation_detected = True
            if (abs_measured_left > (abs_rpm - abs_rpm * 0.05)
                    and abs_measured_right > (abs_rpm - abs_rpm * 0.05)):
                target_reached = True

        self.prev_pwm_left = left_pwm
        self.prev_pwm_right = right_pwm
        return left_pwm, right_pwm, oscillation_detected, target_reached

    def set_motor_output(self, pwm_value, pwm_obj, dir_obj, label=""):
        """Apply one signed motor command to the Cytron PWM/DIR interface."""
        if pwm_value < 0:
            dir_value = 1
            pwm_value = abs(pwm_value)
        else:
            dir_value = 0

        if abs(pwm_value) < self.PWM_MIN:
            pwm_value = 0

        pwm = int(self.clamp(pwm_value, 0, 65535))
        pwm_obj.duty_u16(pwm)
        dir_obj.value(dir_value)
        self._log_debug("%s motor update: pwm=%s dir=%s", label, pwm, "REV" if dir_value else "FWD")

    def stop_motors(self):
        """Force both motor PWM outputs to zero immediately."""
        self.PWM_LEFT.duty_u16(0)
        self.PWM_RIGHT.duty_u16(0)
        self.current_pwm_left = 0
        self.current_pwm_right = 0

    def _ensure_encoder_subsystem(self):
        """Make sure encoder hardware ownership exists before running control."""
        if drivetrain_encoders.initialize():
            return True

        fault = drivetrain_encoders.fault_info() or {"message": "encoder subsystem initialization failed"}
        self._latch_fault("encoder_init_failed", RuntimeError(fault.get("message", "encoder init failed")), extra=fault)
        return False

    def _prepare_running_state(self):
        """Prime encoder timing and PID state before entering the fixed-rate loop.

        Non-obvious rule: entering `running` is treated like a fresh control
        session. The encoder estimator gets a new baseline and both PID
        controllers are reset so no stale history survives from an earlier
        segment.
        """
        if not self._ensure_encoder_subsystem():
            return False

        now_us = time.ticks_us()
        drivetrain_encoders.prime_velocity_estimator(now_us=now_us)
        self.reset_pid_state("start")
        self._next_tick_us = time.ticks_add(now_us, self.CONTROL_PERIOD_US)
        self._last_tick_us = None
        self._loop_overruns = 0
        self._last_summary_ms = time.ticks_ms()
        self._last_loop_measurement = None
        self._last_loop_dt_us = 0
        with self._lock:
            if self._state == self.STATE_STARTING:
                self._set_state_locked(self.STATE_RUNNING, reason="encoder_ready")
        return True

    def _emit_loop_summary(self):
        """Emit low-rate telemetry for tuning and timing visibility.

        The hot path avoids per-tick JSON telemetry because UART traffic and
        allocation pressure can disturb timing on MicroPython. This summary is
        intentionally throttled to once per second.
        """
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, self._last_summary_ms) < self.SUMMARY_PERIOD_MS:
            return

        self._last_summary_ms = now_ms
        measurement = self._last_loop_measurement or {}
        left_p, left_i, left_d = self.pid_left.components
        right_p, right_i, right_d = self.pid_right.components
        telemetry(
            "drivetrain_loop",
            target_rpm=self._last_loop_target_rpm,
            turning_angle=self._last_loop_turning_angle,
            dt_us=self._last_loop_dt_us,
            left_rpm=measurement.get("left_rpm", 0.0),
            right_rpm=measurement.get("right_rpm", 0.0),
            left_delta=measurement.get("left_delta", 0),
            right_delta=measurement.get("right_delta", 0),
            left_pwm=self.current_pwm_left,
            right_pwm=self.current_pwm_right,
            overruns=self._loop_overruns,
            left_p=left_p,
            left_i=left_i,
            left_d=left_d,
            right_p=right_p,
            right_i=right_i,
            right_d=right_d,
        )
        self._loop_overruns = 0

    def _wait_for_next_tick(self):
        """Block until the next scheduled control tick.

        The scheduler is deadline-driven rather than sleep-driven. If a tick
        overruns by a full period, the loop reschedules from "now" so it does
        not try to catch up with burst iterations.
        """
        if self._next_tick_us is None:
            now_us = time.ticks_us()
            self._next_tick_us = time.ticks_add(now_us, self.CONTROL_PERIOD_US)

        while True:
            now_us = time.ticks_us()
            remaining_us = time.ticks_diff(self._next_tick_us, now_us)
            if remaining_us <= 0:
                break
            if remaining_us > 1000:
                time.sleep_ms(remaining_us // 1000)
            else:
                time.sleep_us(remaining_us)

        tick_us = time.ticks_us()
        lateness_us = time.ticks_diff(tick_us, self._next_tick_us)
        if lateness_us >= self.CONTROL_PERIOD_US:
            self._loop_overruns += 1
            self._log_warn("Control loop overrun: lateness_us=%s period_us=%s", lateness_us, self.CONTROL_PERIOD_US)
            self._next_tick_us = time.ticks_add(tick_us, self.CONTROL_PERIOD_US)
        else:
            self._next_tick_us = time.ticks_add(self._next_tick_us, self.CONTROL_PERIOD_US)

        if self._last_tick_us is None:
            dt_us = self.CONTROL_PERIOD_US
        else:
            dt_us = time.ticks_diff(tick_us, self._last_tick_us)
            if dt_us <= 0:
                dt_us = self.CONTROL_PERIOD_US
        self._last_tick_us = tick_us
        self._last_loop_dt_us = dt_us
        return tick_us

    def _handle_direction_reversal(self, tick_us, target_sign):
        """Convert a sign flip into a controlled one-tick zero-output hold.

        Reversal is not treated as "just another PID sample". The controller is
        reset, the estimator is re-primed, and the outputs stay at zero for one
        period so the new direction starts from a clean measurement baseline.
        """
        self.stop_motors()
        self.reset_pid_state("direction_reversal")
        drivetrain_encoders.prime_velocity_estimator(now_us=tick_us)
        self._hold_zero_until_us = time.ticks_add(tick_us, self.CONTROL_PERIOD_US)
        self._last_target_sign = target_sign
        self._log_info("Direction reversal hold: new_sign=%s until_us=%s", target_sign, self._hold_zero_until_us)
        telemetry("drivetrain_reversal", sign=target_sign, hold_until_us=self._hold_zero_until_us)

    def _stop_cycle(self):
        """Finish a requested stop and return the worker to the idle state."""
        self.stop_motors()
        self.reset_pid_state("stop")
        self._next_tick_us = None
        self._last_tick_us = None
        self._last_loop_measurement = None
        with self._lock:
            if self._state == self.STATE_STOPPING:
                self._set_state_locked(self.STATE_IDLE, reason="stop_completed")

    def _worker_loop(self):
        """Persistent worker that owns the drivetrain state machine.

        The loop stays alive for the lifetime of the process. That avoids the
        thread-churn and stale-hardware races from the old start/kill design.
        """
        self._log_info("Persistent drivetrain worker started")
        while True:
            try:
                state = self.state()
                controller = self._driver_controller

                if controller is None or state == self.STATE_IDLE:
                    time.sleep_ms(20)
                    continue

                if state == self.STATE_FAULTED:
                    time.sleep_ms(50)
                    continue

                if state == self.STATE_STOPPING:
                    self._stop_cycle()
                    time.sleep_ms(20)
                    continue

                if state == self.STATE_RECOVERING:
                    time.sleep_ms(20)
                    continue

                if state == self.STATE_STARTING:
                    if self._prepare_running_state():
                        self._emit_loop_summary()
                    time.sleep_ms(10)
                    continue

                tick_us = self._wait_for_next_tick()

                if controller.is_timeout():
                    if controller.timeout_stop():
                        self._log_warn("Command freshness timeout reached; stopping drivetrain")
                    self.terminate_thread()
                    continue

                navigation_params = controller.get_navigation_params()
                signed_rpm_setpoint = navigation_params.get_signed_target_rpm()
                turning_angle = navigation_params.target_angle
                target_sign = self._sign(signed_rpm_setpoint)

                if self.test_mode:
                    signed_rpm_setpoint = 330
                    turning_angle = 0.0
                    target_sign = self._sign(signed_rpm_setpoint)

                if signed_rpm_setpoint == 0:
                    self.terminate_thread()
                    continue

                if self._last_target_sign and target_sign and target_sign != self._last_target_sign:
                    self._handle_direction_reversal(tick_us, target_sign)
                    self._emit_loop_summary()
                    continue

                if self._hold_zero_until_us is not None:
                    if time.ticks_diff(self._hold_zero_until_us, tick_us) > 0:
                        # During the one-tick reversal hold we intentionally do
                        # not run PID. This preserves a clean zero-output gap
                        # between opposite drive directions.
                        self._last_loop_target_rpm = signed_rpm_setpoint
                        self._last_loop_turning_angle = turning_angle
                        self._emit_loop_summary()
                        continue
                    self._hold_zero_until_us = None

                measurement = drivetrain_encoders.sample_velocity(now_us=tick_us)
                if not measurement["valid"]:
                    self._last_loop_measurement = measurement
                    self._last_loop_target_rpm = signed_rpm_setpoint
                    self._last_loop_turning_angle = turning_angle
                    self._emit_loop_summary()
                    continue

                left_pwm, right_pwm, oscillation_detected, target_reached = self.cross_coupled_pid_control(
                    signed_rpm_setpoint,
                    turning_angle,
                    measurement,
                    measurement["dt_us"],
                )

                if self.test_mode and (oscillation_detected or target_reached):
                    self.terminate_thread()
                    continue

                if self.state() != self.STATE_RUNNING or navigation_params.target_rpm == 0:
                    continue

                if self.current_pwm_left != left_pwm:
                    self.set_motor_output(left_pwm, self.PWM_LEFT, self.DIR_LEFT, "Left")
                    self.current_pwm_left = left_pwm
                if self.current_pwm_right != right_pwm:
                    self.set_motor_output(right_pwm, self.PWM_RIGHT, self.DIR_RIGHT, "Right")
                    self.current_pwm_right = right_pwm

                self._last_target_sign = target_sign
                self._last_loop_measurement = measurement
                self._last_loop_target_rpm = signed_rpm_setpoint
                self._last_loop_turning_angle = turning_angle
                self._emit_loop_summary()
            except Exception as exc:
                self._latch_fault("control_loop_failed", exc)
                time.sleep_ms(50)


robot_pid = RobotPID()
