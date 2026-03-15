import unittest

import utime
from src.control.pid_fork import PID


class SimplePidBehaviorTest(unittest.TestCase):
    def setUp(self):
        utime.reset()

    def test_explicit_microsecond_dt_scales_to_seconds(self):
        pid = PID(
            Kp=0.0,
            Ki=2.0,
            Kd=0.0,
            setpoint=1.0,
            scale="us",
            output_limits=(None, None),
            integral_limits=(None, None),
        )

        output = pid(0.0, dt=50_000)

        self.assertAlmostEqual(output, 0.1, places=6)
        self.assertAlmostEqual(pid.components[1], 0.1, places=6)

    def test_integral_limits_are_independent_from_output_limits(self):
        pid = PID(
            Kp=0.0,
            Ki=1.0,
            Kd=0.0,
            setpoint=1.0,
            scale="s",
            output_limits=(-1.0, 1.0),
            integral_limits=(-0.2, 0.2),
        )

        pid(0.0, dt=1.0)
        output = pid(0.0, dt=1.0)

        self.assertAlmostEqual(output, 0.2, places=6)
        self.assertAlmostEqual(pid.components[1], 0.2, places=6)

    def test_set_auto_mode_clamps_seed_to_integral_limits(self):
        pid = PID(
            Kp=0.0,
            Ki=1.0,
            Kd=0.0,
            setpoint=0.0,
            scale="s",
            output_limits=(-1.0, 1.0),
            integral_limits=(-0.25, 0.25),
        )

        pid.auto_mode = False
        pid.set_auto_mode(True, last_output=0.9)

        self.assertEqual(pid.auto_mode, True)
        self.assertAlmostEqual(pid.components[1], 0.25, places=6)

    def test_components_and_reset_preserve_limits(self):
        pid = PID(
            Kp=1.0,
            Ki=1.0,
            Kd=0.0,
            setpoint=2.0,
            scale="s",
            output_limits=(-5.0, 5.0),
            integral_limits=(-0.5, 0.5),
        )

        pid(1.0, dt=1.0)
        self.assertEqual(pid.components, (1.0, 0.5, 0.0))

        pid.reset()

        self.assertEqual(pid.components, (0, 0, 0))
        self.assertEqual(pid.output_limits, (-5.0, 5.0))
        self.assertEqual(pid.integral_limits, (-0.5, 0.5))


if __name__ == "__main__":
    unittest.main()
