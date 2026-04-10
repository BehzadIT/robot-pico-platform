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

### Structured logs
- `loge`, `logw`, `logi`, `logd`, `logv` emit formatted UART logs.
- Current code also emits ANSI-colored output to the USB side, which is an implementation fact to keep in mind during tooling work.

### Telemetry packets
- `telemetry(tag, **data)` emits JSON over UART with fields `t`, `tag`, and `data`.

## Design rule
UART failures must never crash the firmware.
Logging also must never be able to block drivetrain control or websocket
command handling.

## Stability learnings
Recent teleop stabilization work showed that logging pressure was a primary
failure mode on the Pico.

Rules to preserve:
- Keep the logger non-blocking across threads.
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
- drive-log sampling behavior in websocket transport
- new telemetry added in stop/start paths
- any synchronous print/log loop introduced in worker or route code

## Sources
- Code: [logger.py](../src/support/logger.py)
- Code: [logging.py](../src/support/logging.py)
- Code: [navigation.py](../src/transport/navigation.py)
