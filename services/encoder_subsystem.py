"""Long-lived owner for the robot drivetrain encoders.

This module is the robot-facing encoder layer.

It owns both drivetrain encoders across normal drive/stop cycles and keeps all
encoder-specific hardware details in one place:
- which PIO state machines are used
- which pins belong to the left and right encoders
- CPR / gear-ratio conversion
- encoder fault state

The control loop should read data from this subsystem instead of creating
encoder objects on demand.
"""

import time
from log import logd, loge, logi, logw, telemetry
from services.drivetrain_constants import EncoderGeometry, EncoderPins
from services.encoder_pio import PIOQuadratureCounter

ENCODER_TAG = "encoder"


class DrivetrainEncoderSubsystem:
    """Owns both drivetrain encoders and exposes robot-level measurements.

    This is the only long-lived owner of the two PIO encoder counters. The
    drivetrain controller reads through this subsystem instead of creating or
    destroying encoder counters during normal motion commands.
    """
    RIGHT_SM_ID = EncoderPins.RIGHT_SM_ID
    RIGHT_PIN_A = EncoderPins.RIGHT_PIN_A
    RIGHT_PIN_B = EncoderPins.RIGHT_PIN_B
    LEFT_SM_ID = EncoderPins.LEFT_SM_ID
    LEFT_PIN_A = EncoderPins.LEFT_PIN_A
    LEFT_PIN_B = EncoderPins.LEFT_PIN_B

    ENCODER_CPR = EncoderGeometry.ENCODER_CPR
    GEAR_RATIO = EncoderGeometry.GEAR_RATIO

    def __init__(self):
        self._right = None
        self._left = None
        self._initialized = False
        self._fault = None
        self._last_fault = None
        self._last_sample_t_us = None
        self._last_right_count = None
        self._last_left_count = None
        # The control loop reads this cached structure every tick. Reusing one
        # object keeps the measurement path predictable and avoids hot-path
        # dict allocation churn on MicroPython.
        self._velocity_sample = {
            "valid": False,
            "primed": False,
            "t_us": 0,
            "dt_us": 0,
            "right_count": 0,
            "left_count": 0,
            "right_delta": 0,
            "left_delta": 0,
            "right_rpm": 0.0,
            "left_rpm": 0.0,
        }

    def _reset_velocity_estimator(self):
        """Forget previous delta-count history without touching hardware counts."""
        self._last_sample_t_us = None
        self._last_right_count = None
        self._last_left_count = None
        self._velocity_sample["valid"] = False
        self._velocity_sample["primed"] = False
        self._velocity_sample["t_us"] = 0
        self._velocity_sample["dt_us"] = 0
        self._velocity_sample["right_count"] = 0
        self._velocity_sample["left_count"] = 0
        self._velocity_sample["right_delta"] = 0
        self._velocity_sample["left_delta"] = 0
        self._velocity_sample["right_rpm"] = 0.0
        self._velocity_sample["left_rpm"] = 0.0

    def _read_counts(self):
        """Read both counters with one local timestamp.

        The timestamp is taken before the pair of reads so the returned sample
        reflects one control-loop instant as closely as MicroPython allows.
        """
        timestamp_us = time.ticks_us()
        right_count = self._right.read()
        left_count = self._left.read()
        return timestamp_us, right_count, left_count

    def _set_fault(self, code, exc, extra=None):
        self._fault = {
            "code": code,
            "message": str(exc),
            "type": exc.__class__.__name__,
        }
        if extra:
            self._fault.update(extra)
        self._last_fault = dict(self._fault)
        loge(
            "Latched encoder fault code=%s type=%s msg=%s"
            % (self._fault["code"], self._fault["type"], self._fault["message"]),
            ENCODER_TAG,
        )
        telemetry("encoder_fault", **self._fault)

    def _clear_active_fault(self, reason):
        if self._fault is None:
            return
        logi("Clearing active encoder fault (%s)" % reason, ENCODER_TAG)
        telemetry("encoder_fault_cleared", reason=reason, fault=self._fault)
        self._fault = None

    def initialize(self):
        """Initialize both encoder state machines once.

        Returns `True` when the subsystem is ready.
        Returns `False` if initialization previously faulted or if startup fails.
        """
        if self._fault is not None:
            return False
        if self._initialized:
            return True

        try:
            logi(
                "Initializing encoder subsystem: right_sm=%s pins=%s/%s left_sm=%s pins=%s/%s"
                % (
                    self.RIGHT_SM_ID,
                    self.RIGHT_PIN_A,
                    self.RIGHT_PIN_B,
                    self.LEFT_SM_ID,
                    self.LEFT_PIN_A,
                    self.LEFT_PIN_B,
                ),
                ENCODER_TAG,
            )
            self._right = PIOQuadratureCounter(self.RIGHT_SM_ID, self.RIGHT_PIN_A, self.RIGHT_PIN_B)
            self._left = PIOQuadratureCounter(self.LEFT_SM_ID, self.LEFT_PIN_A, self.LEFT_PIN_B)
            self._initialized = True
            self._reset_velocity_estimator()
            telemetry(
                "encoder_initialized",
                right_sm=self.RIGHT_SM_ID,
                right_pins=[self.RIGHT_PIN_A, self.RIGHT_PIN_B],
                left_sm=self.LEFT_SM_ID,
                left_pins=[self.LEFT_PIN_A, self.LEFT_PIN_B],
            )
            return True
        except Exception as exc:
            self.shutdown()
            self._set_fault(
                "init_failed",
                exc,
                extra={
                    "right_sm": self.RIGHT_SM_ID,
                    "right_pins": [self.RIGHT_PIN_A, self.RIGHT_PIN_B],
                    "left_sm": self.LEFT_SM_ID,
                    "left_pins": [self.LEFT_PIN_A, self.LEFT_PIN_B],
                },
            )
            return False

    def is_initialized(self):
        return self._initialized

    def is_faulted(self):
        return self._fault is not None

    def health(self):
        return {
            "initialized": self._initialized,
            "fault": self._fault,
            "last_fault": self._last_fault,
        }

    def fault_info(self):
        return self._fault

    def last_fault_info(self):
        return self._last_fault

    def reset_counts(self):
        """Explicitly zero both encoder counters.

        Resetting counts also resets the velocity estimator baseline because the
        next delta sample must be relative to the new zero point, not the old
        pre-reset counts.
        """
        if not self._initialized:
            return False
        try:
            self._right.set_zero()
            self._left.set_zero()
            self._reset_velocity_estimator()
            logi("Encoder counts reset", ENCODER_TAG)
            telemetry("encoder_reset")
            return True
        except Exception as exc:
            self._set_fault("reset_failed", exc)
            return False

    def snapshot(self):
        """Read the latest raw encoder counts from both tracks.

        This is a raw-count helper for diagnostics and bench validation. The
        live drivetrain loop should prefer `sample_velocity()`.
        """
        if not self._initialized:
            raise RuntimeError("encoder subsystem not initialized")

        timestamp_us, right_count, left_count = self._read_counts()
        return {
            "t_us": timestamp_us,
            "right_count": right_count,
            "left_count": left_count,
        }

    def prime_velocity_estimator(self, now_us=None):
        """Capture a fresh count baseline for the next control-loop tick.

        The drivetrain calls this on startup, stop, and direction reversal so
        the next velocity sample reflects only counts accumulated after the
        transition.
        """
        if not self._initialized:
            raise RuntimeError("encoder subsystem not initialized")

        try:
            sample_t_us, right_count, left_count = self._read_counts()
            if now_us is not None:
                sample_t_us = now_us
            self._last_sample_t_us = sample_t_us
            self._last_right_count = right_count
            self._last_left_count = left_count
            self._velocity_sample["valid"] = False
            self._velocity_sample["primed"] = True
            self._velocity_sample["t_us"] = sample_t_us
            self._velocity_sample["dt_us"] = 0
            self._velocity_sample["right_count"] = right_count
            self._velocity_sample["left_count"] = left_count
            self._velocity_sample["right_delta"] = 0
            self._velocity_sample["left_delta"] = 0
            self._velocity_sample["right_rpm"] = 0.0
            self._velocity_sample["left_rpm"] = 0.0
            logd("Velocity estimator primed: t_us=%s right=%s left=%s" % (
                sample_t_us,
                right_count,
                left_count,
            ), ENCODER_TAG)
            return self._velocity_sample
        except Exception as exc:
            self._set_fault("prime_failed", exc)
            raise

    def sample_velocity(self, now_us=None):
        """Update cached delta-count velocity for the current control tick.

        This is the non-blocking measurement path used by the drivetrain loop.
        The returned dict is reused on each call; callers should treat it as a
        read-only snapshot for the current iteration only.
        """
        if not self._initialized:
            raise RuntimeError("encoder subsystem not initialized")

        try:
            sample_t_us, right_count, left_count = self._read_counts()
            if now_us is not None:
                sample_t_us = now_us

            if self._last_sample_t_us is None:
                # The first sample after a reset/prime only establishes the
                # baseline. Returning `valid=False` prevents the caller from
                # running PID on a zero-length or undefined delta window.
                self._last_sample_t_us = sample_t_us
                self._last_right_count = right_count
                self._last_left_count = left_count
                self._velocity_sample["valid"] = False
                self._velocity_sample["primed"] = True
                self._velocity_sample["t_us"] = sample_t_us
                self._velocity_sample["right_count"] = right_count
                self._velocity_sample["left_count"] = left_count
                self._velocity_sample["dt_us"] = 0
                self._velocity_sample["right_delta"] = 0
                self._velocity_sample["left_delta"] = 0
                self._velocity_sample["right_rpm"] = 0.0
                self._velocity_sample["left_rpm"] = 0.0
                return self._velocity_sample

            dt_us = time.ticks_diff(sample_t_us, self._last_sample_t_us)
            if dt_us <= 0:
                raise RuntimeError("encoder dt_us must be positive")

            right_delta = right_count - self._last_right_count
            left_delta = left_count - self._last_left_count
            # ENCODER_CPR is already the fully decoded quadrature count at the
            # motor shaft. Multiply only by gear ratio to get output-shaft
            # counts/rev; do not multiply by 4 again here.
            rev_scale = 60_000_000.0 / (self.ENCODER_CPR * self.GEAR_RATIO * dt_us)

            self._last_sample_t_us = sample_t_us
            self._last_right_count = right_count
            self._last_left_count = left_count
            self._velocity_sample["valid"] = True
            self._velocity_sample["primed"] = True
            self._velocity_sample["t_us"] = sample_t_us
            self._velocity_sample["dt_us"] = dt_us
            self._velocity_sample["right_count"] = right_count
            self._velocity_sample["left_count"] = left_count
            self._velocity_sample["right_delta"] = right_delta
            self._velocity_sample["left_delta"] = left_delta
            self._velocity_sample["right_rpm"] = right_delta * rev_scale
            self._velocity_sample["left_rpm"] = left_delta * rev_scale
            return self._velocity_sample
        except Exception as exc:
            self._set_fault("read_failed", exc)
            raise

    def measure_rpms_window(self, interval_ms=50):
        """Measure track RPM using the current blocking sample-window method.

        This helper remains available for bench comparison and debugging, but
        the live drivetrain loop should use `sample_velocity()` instead.
        """
        if not self._initialized:
            raise RuntimeError("encoder subsystem not initialized")

        try:
            start = self.snapshot()
            time.sleep_ms(interval_ms)
            end = self.snapshot()

            dt_ms = time.ticks_diff(end["t_us"], start["t_us"]) / 1000.0
            if dt_ms <= 0:
                raise RuntimeError("encoder dt must be positive")

            right_delta = end["right_count"] - start["right_count"]
            left_delta = end["left_count"] - start["left_count"]
            rev_scale = 60000.0 / (self.ENCODER_CPR * self.GEAR_RATIO * dt_ms)
            right_rpm = right_delta * rev_scale
            left_rpm = left_delta * rev_scale

            return {
                "t_us": end["t_us"],
                "dt_ms": dt_ms,
                "right_count": end["right_count"],
                "left_count": end["left_count"],
                "right_delta": right_delta,
                "left_delta": left_delta,
                "right_rpm": right_rpm,
                "left_rpm": left_rpm,
            }
        except Exception as exc:
            self._set_fault("read_failed", exc)
            raise

    def shutdown(self):
        """Shut down both encoder counters.

        This is intended for explicit teardown or fault cleanup, not normal
        drive/stop transitions.
        """
        for counter in (self._right, self._left):
            if counter is None:
                continue
            try:
                counter.deinit()
            except Exception as exc:
                logw("Ignoring encoder shutdown error: %s" % exc, ENCODER_TAG)

        self._right = None
        self._left = None
        self._reset_velocity_estimator()
        if self._initialized:
            logd("Encoder subsystem shut down", ENCODER_TAG)
        self._initialized = False

    def recover(self):
        """Perform a full encoder-owner teardown and reinitialization.

        Recovery intentionally uses the subsystem boundary instead of trying to
        revive one state machine in place. That keeps PIO ownership, fault
        state, and estimator state consistent after a failure.
        """
        logi("Encoder recovery requested", ENCODER_TAG)
        telemetry("encoder_recover_requested", fault=self._fault, last_fault=self._last_fault)

        self.shutdown()
        self._clear_active_fault("recover_requested")

        if not self.initialize():
            fault = self._fault or {"message": "encoder recovery failed"}
            raise RuntimeError(fault.get("message", "encoder recovery failed"))

        self._reset_velocity_estimator()
        logi("Encoder recovery completed", ENCODER_TAG)
        telemetry("encoder_recovered", last_fault=self._last_fault)
        return True


drivetrain_encoders = DrivetrainEncoderSubsystem()
