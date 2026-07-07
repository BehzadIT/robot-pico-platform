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
current job is to prove the fast C++ bench loop plus the no-network RTOS
skeleton: USB stdio, picotool reset support, startup logs, pinned FreeRTOS
service/control tasks, and deterministic heartbeat output without CYW43,
network transport, motor output, encoder ownership, or watchdog-controlled
motion.

## Target names
- `robot_pico_firmware`: current migrated Pico C++ firmware path with runtime
  scaffolding, FreeRTOS, transport, hardware wrappers, and control-core wiring.
- `robot_pico_control_no_wifi`: clean-start Pico SDK app used to rebuild the
  no-network control firmware in small verified stages.

Keep these targets parallel until the clean-start path has independently proven
boot, RTOS task placement, UART handoff, command admission, stale-stop behavior,
and motor safety on the bench.

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
