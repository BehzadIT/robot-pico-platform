# Firmware Runtime Plan

## Intended runtime split
- Target board: Raspberry Pi Pico 2 W (`PICO_BOARD=pico2_w`)
- Core 0: FreeRTOS service side
  - Wi-Fi stack service
  - Mongoose HTTP/WebSocket server
  - joystick UART ingress and framed parsing
  - logging drain
  - diagnostics/status endpoints
- Core 1: deterministic control domain
  - teleop semantics
  - command admission
  - drivetrain state machine
  - encoder truth
  - PID and actuator output
  - stop/recover/fault/watchdog authority

## Current state
This tree now contains the Pico SDK firmware bring-up:
- multicore boot with the control runtime on Core 1
- FreeRTOS task startup on Core 0
- Wi-Fi join through `cyw43`
- Mongoose HTTP/WebSocket server scaffolding
- Pico PWM/DIR and watchdog wrappers
- a Pico SDK PIO quadrature counter port used by the persistent encoder owner

It is still not bench-validated. The C++ runtime should continue to be treated
as the migration target until the hardware parity checklist passes.

## Clean app
The clean-start app lives in `apps/pico_control_no_wifi/` and builds as
`robot_pico_control_no_wifi`.

That target is intentionally separate from the migrated runtime sources. Its
initial job is only to prove the fast C++ bench loop: USB stdio, picotool reset
support, startup logs, and heartbeat output without CYW43, network transport,
motor output, encoder ownership, or watchdog-controlled motion.

## Config split
- Tracked non-secret config lives in `cpp/include/robot/config/config.hpp`.
- Local secrets should be provided through a gitignored `secrets.hpp` copied
  from `cpp/include/robot/config/secrets.example.hpp`.

## Dependency pins
- FreeRTOS-Kernel: vendored under `cpp/third_party/freertos-kernel`
- Mongoose: vendored under `cpp/third_party/mongoose`
- Pico SDK: external via `PICO_SDK_PATH`

## Board notes
- The firmware tree is configured and documented for `pico2_w`.
- Pico W-era documentation still applies conceptually for CYW43 Wi‑Fi and lwIP,
  but board-specific CMake and SDK configuration should follow the Pico 2 W
  target, not `pico_w`.
