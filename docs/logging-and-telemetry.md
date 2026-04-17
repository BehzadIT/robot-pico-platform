# Logging And Telemetry

## Purpose
Document the Pico logging architecture and the log parsing contract expected by the ESP32 bridge and bench tools.

## Current status
- `Implemented`: `print()` output is mirrored to UART while preserving normal USB REPL output.
- `Implemented`: structured logs use one stdout-oriented stream that stays readable by humans and parsable by tools.
- `Project note`: preserving IDE compatibility is a hard requirement.
- `Migration target`: in the C++ runtime, logging remains a service-side drain
  and must not back-pressure the control domain.
- `Migration status`: the C++ runtime now has service/control separation in
  code, but the Pico SDK transport/logging path is still pending full hardware
  validation on the Pico 2 W target.

## Behavior
### `print()`
- USB REPL output should remain unchanged.
- UART copy is prefixed with boot-relative ticks.
- The normal USB `print()` / REPL stdout path must not be routed through the
  shared logger lock. Bench verification showed that lock-coupling USB stdout
  to the custom logger path made `mpremote` raw-REPL entry unreliable.

### Structured logs
- `loge`, `logw`, `logi`, `logd`, `logv` emit formatted logs.
- Keep the line format stable so tools can parse the same stream that humans
  read:
  - `<LEVEL> (<ticks_ms>) <tag>: <message>`
- If the tag is empty, omit the tag segment cleanly rather than emitting a
  blank placeholder.
- Prefer key-value tails in the message body when a tool needs to parse extra
  fields later.

### Emission policy
- `Immediate`: faults and safety events log as soon as they occur.
- `Transition`: state machines log only when the value changes.
- `Rate-limited`: repeated warnings and retry progress use caller-owned rate
  limiting so they do not spam the console.
- `Sampled`: periodic summaries are caller-owned snapshots, used only when a
  bounded heartbeat or debug summary is genuinely useful.
- Sampling is not a logger feature. The owning subsystem decides whether a
  sampled summary is enabled and when it stops.

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

## RTOS boundary
- Logging transport belongs on the RTOS/service side in the C++
  migration.
- The control domain may emit events or sampled status, but it must not wait on
  a logger task, socket, UART drain, or serialization step.
- If the service-side logger queue is full, the system should drop or coalesce
  logs rather than stall control.
- Watchdog progress must reflect control progress, not successful logging.

In the current C++ tree this boundary is represented by:
- service-side runtime and transport code under `cpp/firmware/src/platform` and
  `cpp/firmware/src/transport`
- control-side event emission under `cpp/src/control` and `cpp/src/runtime`

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
- any synchronous print/log loop introduced in worker or route code

## Sources
- Code: [logger.py](../src/support/logger.py)
- Code: [logging.py](../src/support/logging.py)
- Code: [mobile_app_transport.py](../src/transport/mobile_app_transport.py)
