import utime


def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value

class PID(object):
    """A simple PID controller.

    This local helper is used in the drivetrain hot path, so its timing rules
    need to stay explicit. When the caller provides `dt`, that value is the
    source of truth for the control interval.
    """

    def __init__(
        self,
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        setpoint=0,
        sample_time=None,
        scale='ms',
        output_limits=[None, None],
        integral_limits=[None, None],
        auto_mode=True,
        proportional_on_measurement=False,
        error_map=None
    ):
        """
        Initialize a new PID controller.
        (see your previous docstring for parameter details)
        """
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        def get_scale(x):
            return {
                's' : 'time',
                'ms': 'ticks_ms',
                'us': 'ticks_us',
                'ns': 'time_ns',
                'cpu':'ticks_cpu'
            }.get(x, 'time')
        self.scale = get_scale(scale)

        def get_unit(x):
            return {
                's' : 1,
                'ms': 1e-3,
                'us': 1e-6,
                'ns': 1e-9,
                'cpu':1  # cpu returns value as-is, no real time
            }.get(x, 1)
        self.unit = get_unit(scale)

        if hasattr(utime, self.scale) and callable(func := getattr(utime, self.scale)):
            self.time = func

        self._min_output, self._max_output = None, None
        self._min_integral, self._max_integral = None, None
        self._auto_mode = auto_mode
        self.proportional_on_measurement = proportional_on_measurement
        self.error_map = error_map

        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_time = None
        self._last_output = None
        self._last_input = None

        self.output_limits = output_limits
        self.integral_limits = integral_limits
        self.reset()

    def _dt_to_seconds(self, dt):
        """
        Convert a time interval dt from the configured scale to seconds.
        For 'cpu', returns dt as-is (user must interpret).
        """
        if self.scale == 'ticks_cpu':
            return dt
        return float(dt) * self.unit

    def __call__(self, input_, dt=None):
        """
        Update the PID controller.
        (see your previous docstring for parameter details)
        """
        if not self.auto_mode:
            return self._last_output

        now = self.time()
        if dt is None:
            raw_dt = utime.ticks_diff(now, self._last_time) if (self._last_time is not None) else 0
            dt = raw_dt if raw_dt != 0 else 1e-16
        elif dt <= 0:
            raise ValueError('dt has negative value {}, must be positive'.format(dt))

        # The drivetrain loop runs in microseconds, but the PID math uses
        # seconds internally so Ki/Kd stay in conventional units.
        dt_sec = self._dt_to_seconds(dt)

        if self.sample_time is not None and dt_sec < self.sample_time and self._last_output is not None:
            return self._last_output

        error = self.setpoint - input_
        d_input = input_ - (self._last_input if (self._last_input is not None) else input_)

        if self.error_map is not None:
            error = self.error_map(error)

        if not self.proportional_on_measurement:
            self._proportional = self.Kp * error
        else:
            self._proportional -= self.Kp * d_input

        self._integral += self.Ki * error * dt_sec
        self._integral = _clamp(self._integral, self.integral_limits)

        self._derivative = -self.Kd * d_input / dt_sec if dt_sec != 0 else 0

        output = self._proportional + self._integral + self._derivative
        output = _clamp(output, self.output_limits)

        self._last_output = output
        self._last_input = input_
        self._last_time = now

        return output

    def __repr__(self):
        return (
            '{self.__class__.__name__}('
            'Kp={self.Kp!r}, Ki={self.Ki!r}, Kd={self.Kd!r}, '
            'setpoint={self.setpoint!r}, sample_time={self.sample_time!r}, '
            'output_limits={self.output_limits!r}, auto_mode={self.auto_mode!r}, '
            'proportional_on_measurement={self.proportional_on_measurement!r},'
            'error_map={self.error_map!r}'
            ')'
        ).format(self=self)

    @property
    def components(self):
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self):
        return self.Kp, self.Ki, self.Kd

    @tunings.setter
    def tunings(self, tunings):
        self.Kp, self.Ki, self.Kd = tunings

    @property
    def auto_mode(self):
        return self._auto_mode

    @auto_mode.setter
    def auto_mode(self, enabled):
        self.set_auto_mode(enabled)

    def set_auto_mode(self, enabled, last_output=None):
        """Enable/disable automatic control mode.

        When re-entering auto mode, integral state is seeded from the last
        output and clamped to the integral limits so re-enable does not cause
        an immediate windup jump.
        """
        if enabled and not self._auto_mode:
            self.reset()
            self._integral = last_output if (last_output is not None) else 0
            self._integral = _clamp(self._integral, self.integral_limits)
        self._auto_mode = enabled

    @property
    def output_limits(self):
        return self._min_output, self._max_output

    @property
    def integral_limits(self):
        return self._min_integral, self._max_integral

    @output_limits.setter
    def output_limits(self, limits):
        if limits is None:
            self._min_output, self._max_output = None, None
            return
        min_output, max_output = limits
        if (None not in limits) and (max_output < min_output):
            raise ValueError('lower limit must be less than upper limit')
        self._min_output = min_output
        self._max_output = max_output
        self._integral = _clamp(self._integral, self.output_limits)
        self._last_output = _clamp(self._last_output, self.output_limits)


    @integral_limits.setter
    def integral_limits(self, limits):
        if limits is None:
            self._min_integral, self._max_integral = None, None
            return
        min_integral, max_integral = limits
        if (None not in limits) and (max_integral < min_integral):
            raise ValueError('lower limit must be less than upper limit')
        self._min_integral = min_integral
        self._max_integral = max_integral
        self._integral = _clamp(self._integral, self.integral_limits)

    def reset(self):
        """Clear P/I/D history while preserving configured limits and gains."""
        self._proportional = 0
        self._integral = 0
        self._derivative = 0
        self._integral = _clamp(self._integral, self.integral_limits)
        self._last_time = self.time()
        self._last_output = None
        self._last_input = None
