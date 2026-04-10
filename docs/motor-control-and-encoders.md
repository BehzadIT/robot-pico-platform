# Motor Control And Encoders

## Purpose
Describe the implemented motor-control logic, RPM interpretation, and encoder handling.

## Current status
- `Implemented`: dual-track PID-based control loop with PIO encoder reading.
- `Implemented`: signed target RPM is converted into direction plus absolute speed.
- `Needs verification`: actual left encoder pins on the current robot.

## Motor control behavior
- Control loop uses target RPM and target angle state from the navigation controller.
- The active WebSocket route now exposes signed throttle / turn intent and converts that into internal RPM / direction / angle targets before the control loop runs.
- HTTP legacy payloads still expose target RPM, target direction, and target angle directly.
- PID output is converted to signed PWM and then mapped to Cytron PWM/DIR outputs.
- Stop currently terminates the control thread and stops motors.

## Encoder behavior
- PIO quadrature counter is used for dual encoders.
- Current code assumes full quadrature decoding with `ENCODER_CPR = 64`.
- `GEAR_RATIO = 30` yields `1920 counts/output rev`.

## Important mismatch
The docs and wiring notes point to left encoder pins `GP12/13`, while one current implementation path uses `GP20/21`. This is a high-priority verification item.

## Sources
- Code: [robot_cross_coupled_pid.py](../services/robot_cross_coupled_pid.py)
- Code: [dual_rpm_pio_test.py](../services/dual_rpm_pio_test.py)
- Source file: [Pololu_37D_Encoder_Spec_and_RPM_Test_Code (1).txt](../docs/Pololu_37D_Encoder_Spec_and_RPM_Test_Code%20%281%29.txt)
- Source file: [wiring reference.txt](../docs/wiring%20reference.txt)
