# Logging And Telemetry

## Purpose
Document the Pico logging architecture and the telemetry contract expected by the ESP32 bridge and bench tools.

## Current status
- `Implemented`: `print()` output is mirrored to UART while preserving normal USB REPL output.
- `Implemented`: structured logs and telemetry use UART-first behavior.
- `Project note`: preserving IDE compatibility is a hard requirement.

## Behavior
### `print()`
- USB REPL output should remain unchanged.
- UART copy is prefixed with boot-relative ticks.
- The normal USB `print()` / REPL stdout path must not be routed through the
  shared logger lock. Bench verification showed that lock-coupling USB stdout
  to the custom logger path made `mpremote` raw-REPL entry unreliable.

### Structured logs
- `loge`, `logw`, `logi`, `logd`, `logv` emit formatted UART logs.
- Current code also emits ANSI-colored output to the USB side, which is an implementation fact to keep in mind during tooling work.

### Telemetry packets
- `telemetry(tag, **data)` emits JSON over UART with fields `t`, `tag`, and `data`.

### Controller teleop diagnostics
- `joystick_uart_receiver.py` now owns the full controller pipeline and logs
  controller mode transitions instead of every loop iteration so observability
  stays useful without adding high-rate log pressure.
- The worker reports transitions such as `inactive -> driving` or
  `driving -> armed_neutral` and records stop reasons like stale snapshot,
  disconnect, neutral throttle, or drivetrain rejection.
- This is intentionally transition-based because the same worker republishes
  drive commands at a fixed cadence even when controller input is steady.

## Design rule
UART failures must never crash the firmware.
Logging also must never be able to block drivetrain control or websocket
command handling.

## Stability learnings
Recent teleop stabilization work showed that logging pressure was a primary
failure mode on the Pico.

Rules to preserve:
- Keep the logger non-blocking across threads.
- Serialize only the custom UART log transport. Do not serialize normal USB
  REPL/stdout behind the shared logger lock.
- Sample high-rate transport logs instead of emitting one line per drive frame.
- Keep stop/restart paths quiet unless extra logging is needed for a specific
  safety investigation.
- Treat added UART/USB log volume as a control-path change, not just an
  observability change.

Symptoms that previously pointed at logging trouble:
- drivetrain worker appearing to stop immediately after startup
- websocket commands apparently accepted but no further drivetrain progress
- watchdog resets around drive/stop boundaries without a clear control fault

If teleop becomes unstable again, inspect:
- global logger lock behavior
- any change that routes normal USB REPL/stdout through the shared logger lock
- drive-log sampling behavior in websocket transport
- new telemetry added in stop/start paths
- any synchronous print/log loop introduced in worker or route code

## Sources
- Code: [logger.py](../src/support/logger.py)
- Code: [logging.py](../src/support/logging.py)
- Code: [mobile_app_transport.py](../src/transport/mobile_app_transport.py)
