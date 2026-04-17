# Pico Firmware Architecture

## Purpose
Describe the current Raspberry Pi Pico firmware structure and runtime responsibilities.

The current production and migration board target is Raspberry Pi Pico 2 W.
Historical notes that say only "Pico" or "Pico W" reflect earlier bring-up
context unless the text explicitly discusses older runtime behavior.

## Current status
- `Implemented`: MicroPython application in `robot-pico-platform` owns Wi-Fi connection, control routes, motor control, and logging.
- `In progress`: staged C++ migration skeleton under `cpp/` for protocol/control
  parity tests and the future Pico SDK runtime.
- `Implemented in C++ tree, pending hardware validation`: persistent encoder
  owner, drivetrain state machine, control-core-owned controller UART path,
  Pico SDK PWM/watchdog wrappers, Pico SDK PIO quadrature counter port,
  dual-core FreeRTOS bring-up, and Mongoose transport scaffolding.

## Entry flow
- repo-root `main.py` delegates into `src/app/`.
- The runtime enables UART print mirroring.
- It creates a Microdot app.
- It registers routes.
- It starts the joystick UART receiver, which now owns the full controller
  pipeline: UART read, snapshot storage, mapping, and fixed-cadence drive
  republish.
- It reads tracked runtime settings from `settings/config.py`.
- It reads local Wi-Fi credentials from `settings/secrets.py`.
- It connects to Wi-Fi.
- It starts the HTTP server on port `80`.

## Key modules
- `src/transport/mobile_app_transport.py`: mobile-app-facing HTTP and WebSocket control endpoints
- `src/transport/joystick_uart_receiver.py`: single-worker controller pipeline for UART receive, snapshot freshness, mapping, and drive republish
- `src/control/drive_command_manager.py`: drive command admission, source ownership, and drive target handoff
- `src/control/controller_input_mapping.py`: controller trigger/stick mapping plus hysteresis and dead-zone rules
- `src/control/drivetrain_controller.py`: drive loop, PWM output, encoder reads
- `src/support/logger.py`: REPL-safe logging implementation
- `src/platform/wifi_connection.py`: station-mode Wi-Fi join logic

## Root layout
- `main.py`: Pico firmware bootstrap
- `src/`: project-owned runtime code
- `lib/`: runtime dependencies deployed to the Pico
- `settings/`: Pico-side configuration package
- `docs/`: reference material
- `experiments/`: non-production tuning and exploratory scripts
- `cpp/`: staged C++ migration tree for host-side parity logic, docs-aligned
  boundaries, and the future Pico SDK runtime

## Board target
- Current hardware target for the Pico SDK migration: Raspberry Pi Pico 2 W
- Expected firmware CMake board setting: `PICO_BOARD=pico2_w`
- Wi‑Fi and CYW43 stack assumptions in this repo should be interpreted through
  the Pico 2 W board definition unless a doc explicitly calls out older Pico W
  behavior

## C++ migration target
The migration preserves the same subsystem ownership:
- Pico remains the motion and safety authority.
- ESP32 remains controller and telemetry transport side only.
- Android remains operator input only.

The target runtime splits into two execution domains:
- RTOS/service side:
  - Wi-Fi stack service
  - HTTP/WebSocket session handling
  - UART byte ingress and framing
  - logging/telemetry drain
  - diagnostics
- control domain:
  - controller mapping and teleop semantics
  - command admission and freshness policy
  - drivetrain state machine and fixed-cadence loop
  - encoder sampling
  - PID and actuator output
  - fault/recover/watchdog authority

The preferred seam is narrow: service code parses and packages transport data,
then the control domain decides whether the robot may move.

The current C++ runtime layout under `cpp/` now reflects that split directly:
- `include/robot/control` and `src/control`: control-domain policy, drivetrain
  loop, PID, and controller mapping
- `include/robot/hardware` and `src/hardware`: host-buildable ownership
  boundaries plus Pico-specific hardware wrappers in `cpp/firmware/src/hardware`
- `include/robot/runtime` and `src/runtime`: bounded inter-core mailbox and
  service/control runtime composition
- `cpp/firmware`: Pico SDK boot, dual-core FreeRTOS task bootstrap, Mongoose
  server, and Pico-only hardware/runtime code

## Notes
The Pico firmware is currently both the real-time controller and the live control server. That is a current implementation fact even if future architecture changes move command routing elsewhere.
The dual-core Pico SDK runtime keeps the same ownership split: core 1 runs the
fixed-cadence control task and owns drivetrain safety, encoder truth, and final
actuator output; core 0 runs the RTOS/service task and owns Wi-Fi, transport,
diagnostics, and logging drain. The boundary remains control versus service
ownership, not "single-core control and externalized service".
The Pico also now owns controller-to-drive mapping for Xbox input received from
the ESP32 transport. The ESP32 forwards normalized snapshots plus periodic
heartbeat frames; the Pico's controller worker decides whether that input is
`driving`, `armed_neutral`, or `inactive`, republishes fresh drive commands at
a fixed cadence while input is valid, and requests stop when input becomes
stale, neutral, or disconnected.
The shared upload workflow targets the real firmware paths (`main.py`, `src/`, `lib/`, `settings/`) via `tools/pico_deploy.py` and root-relative exclusions from `.picodeployignore`, rather than a mirrored deploy tree.
Drivetrain watchdog settings are owned by `settings/config.py` and are rolled
out in stages. The current safe pattern is to defer watchdog arming until after
the first clean stop cycle instead of arming immediately at startup.
The C++ port retains that staged watchdog rule in
`cpp/include/robot/control/drivetrain_controller.hpp` and
`cpp/include/robot/config/config.hpp`.

## Sources
- Code: [main.py](../main.py)
- Code: [mobile_app_transport.py](../src/transport/mobile_app_transport.py)
- Code: [joystick_uart_receiver.py](../src/transport/joystick_uart_receiver.py)
- Code: [drive_command_manager.py](../src/control/drive_command_manager.py)
- Code: [controller_input_mapping.py](../src/control/controller_input_mapping.py)
- Code: [drivetrain_controller.py](../src/control/drivetrain_controller.py)
- Doc: [runtime-boundaries.md](./runtime-boundaries.md)
