# Pico C++ Migration Skeleton

## Purpose
This tree is the staged C++ migration target for the Pico firmware.

It preserves the current project boundary:
- Pico remains the control and safety authority.
- RTOS/service code may receive and parse input.
- Only the control domain may decide whether the robot is allowed to move.

The active on-device target for this tree is the Raspberry Pi Pico 2 W. Build
and flash instructions should use `PICO_BOARD=pico2_w` unless a different
board migration is being done intentionally.

## Current scope
- `include/` and `src/` now contain the host-buildable control core:
  protocol framing, command admission, controller UART mapping, persistent
  encoder ownership, drivetrain state machine, PID, and bounded inter-core
  handoff types.
- `tests/` contains parity-oriented host tests for logic that should survive the
  runtime rewrite unchanged.
- `firmware/` contains the Pico SDK runtime tree with FreeRTOS service-side
  bring-up, Mongoose transport scaffolding, Pico motor/watchdog drivers, and a
  PIO quadrature counter port.

## Build modes
- Default host build:
  - `cmake -S cpp -B cpp/build-host`
  - `cmake --build cpp/build-host`
  - `ctest --test-dir cpp/build-host --output-on-failure`
- Firmware build:
  - copy `cpp/include/robot/config/secrets.example.hpp` to
    `cpp/include/robot/config/secrets.hpp`
  - `source tools/pico_sdk_env.sh`
  - or export `PICO_SDK_PATH=/home/behzad/Projects/pico-sdk`
  - `cmake -S cpp -B cpp/build-pico -DROBOT_PICO_ENABLE_FIRMWARE=ON -DPICO_BOARD=pico2_w`
  - `cmake --build cpp/build-pico`

The firmware build links the vendored `FreeRTOS-Kernel` and `Mongoose` trees
from `cpp/third_party/`; it does not download dependencies at configure time.

## Boundary rule
- RTOS/service side: Wi-Fi, sockets, HTTP/WebSocket I/O, UART byte ingress,
  telemetry drain, diagnostics.
- Control side: joystick semantics, command admission, stale-input stop, drive
  target ownership, encoder truth, PID, actuator output, watchdog progress.

The handoff must stay narrow: transport packages data, control decides motion.
