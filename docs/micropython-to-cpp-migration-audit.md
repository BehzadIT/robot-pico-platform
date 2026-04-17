# MicroPython To C++ Migration Audit

## Purpose
Audit the active MicroPython Pico firmware against the staged Pico SDK / C++
runtime for Raspberry Pi Pico 2 W, record what already migrated, and make the
remaining gaps explicit enough that later porting work does not need to
re-decide ownership, logging transport, or safety boundaries.

This document is the migration source of truth for:
- behavior parity between `src/` and `cpp/`;
- missing runtime features, diagnostics, and tools; and
- the logger migration contract for the Pico 2 W C++ runtime.

## Baseline
- `src/` remains the current behavior source of truth.
- `cpp/` is the migration target, not parity-complete production firmware.
- Pico remains the motion, timing, and safety authority.
- ESP32 remains a transport-side consumer or forwarder only; it must not regain
  control ownership during the logger migration.

Status labels used below:
- `Ported`: the C++ tree carries the same responsibility with no known
  behavior-class gap.
- `Partial`: the core behavior exists, but important diagnostics, lifecycle, or
  edge-case behavior is still missing.
- `Missing`: no meaningful C++ equivalent exists yet.
- `Deferred`: intentionally not migrated yet; the omission is known.

## Logger Migration Contract
The MicroPython logger is a migration gap, not a removed requirement.

Target contract for the C++ runtime:
- Application-owned manual UART logging is retired.
- The logger emits to one stdio sink only.
- Pico SDK / CMake decides whether that sink is USB stdio, UART stdio, or both.
- The C++ runtime exposes leveled helpers equivalent in intent to
  `loge/logw/logi/logd/logv`.
- The formatter owns level prefixing, tag formatting, optional tick timestamps,
  and optional ANSI styling.
- There is no separate telemetry API or second stream; parseable fields stay in
  the same log line as the human-readable text.
- ESP32 integration later should consume standard stdio-backed UART output when
  UART stdio is enabled, rather than reviving a repo-owned manual UART mirror.

Non-goals:
- no dual-path custom `print()` mirror like the MicroPython implementation;
- no logger-owned routing split between USB and manual UART;
- no control-loop blocking on logging formatting; and
- no separate telemetry stream or helper API; and
- no movement of motion or safety policy into service/logging code.

## Audit Matrix
| MicroPython source | Current responsibility | C++ counterpart | Status | Gap summary | Intended C++ ownership |
| --- | --- | --- | --- | --- | --- |
| `src/app/runtime.py` | Boot composition, UART print mirror enable, boot reset-cause log, joystick worker startup, Wi-Fi join, server start | `cpp/firmware/src/app/pico_firmware_app.cpp`, `cpp/firmware/src/platform/pico_service_core.cpp` | `Partial` | C++ boots the service/control runtimes, but it does not yet emit boot breadcrumbs, reset-cause diagnostics, joystick-pipeline startup status, or explicit server-start logs. | Boot lifecycle in `cpp/firmware/src/app`; service lifecycle logs in `cpp/firmware/src/platform` |
| `src/support/logger.py` | Leveled logger, print mirroring, tick prefixing, ANSI styling, log-level filtering, JSON telemetry, UART failure isolation | No repo-owned C++ equivalent yet; only `stdio_init_all()` plus SDK stdio backend selection | `Missing` | The old logger API and formatting behavior do not exist in `cpp/`; switching stdio backend alone does not recreate logger semantics. | New C++ logger facade in `cpp/include/robot/support` and `cpp/src/support`; stdio backend selection remains in `cpp/firmware/CMakeLists.txt` |
| `src/support/logging.py` | Import facade and grouped logger surface | No C++ equivalent | `Missing` | There is no standardized C++ helper surface for repeated call sites. | Same new support-layer logger facade |
| `src/platform/wifi_connection.py` | Wi-Fi retry/backoff state machine, status transition logs, failure classification, IP reporting, telemetry | `cpp/firmware/src/platform/pico_service_core.cpp`, `cpp/include/robot/config/config.hpp` | `Partial` | C++ has config fields for retry/backoff, but current runtime performs one blocking connect call and silently drops the task on failure. No transition logs, retry loop, failure classification, or IP report are present. | Wi-Fi lifecycle in `cpp/firmware/src/platform`; policy values in `cpp/include/robot/config` |
| `src/transport/mobile_app_transport.py` | HTTP/WebSocket routes, protocol parsing, ack/error mapping, sampled transport logs, stop/recover wait semantics, route-level recoverable exception handling | `cpp/firmware/src/transport/mongoose_http_server.cpp`, `cpp/src/runtime/service_core_runtime.cpp` | `Partial` | C++ transport accepts drive/stop/recover and returns completions, but it has thinner parsing, fewer protocol distinctions, no route-level logs, no recoverable exception breadcrumbs, and less explicit error mapping. | Transport and service logs in `cpp/firmware/src/transport`; queue handoff in `cpp/src/runtime` |
| `src/transport/routes.py` | Route registration composition | Mongoose listener setup inside `cpp/firmware/src/transport/mongoose_http_server.cpp` | `Ported` | Route registration exists in different form; no separate gap beyond missing transport diagnostics. | `cpp/firmware/src/transport` |
| `src/transport/joystick_uart_receiver.py` | Dedicated controller UART worker, framed receive loop, snapshot status, mode transitions, controller stop reasons, diagnostics, input-delta reporting | `cpp/src/control/controller_uart_pipeline.cpp`, `cpp/src/runtime/control_core_runtime.cpp`, control-core UART read in `cpp/firmware/src/app/pico_firmware_app.cpp` | `Partial` | The control-side ingest/mapping/republish logic migrated, but the status snapshots, lifecycle logs, stop-reason breadcrumbs, and operator diagnostics did not. | Control semantics stay in `cpp/src/control`; control/runtime status export in `cpp/src/runtime`; UART byte ingress stays in the Pico control/runtime boundary |
| `src/protocol/joystick_transport.py` and `src/protocol/joystick_snapshot.py` | Framing, CRC, neutral joystick DTO | `cpp/include/robot/protocol/joystick_transport.hpp`, `cpp/include/robot/protocol/joystick_snapshot.hpp`, implementation in `cpp/src/protocol` | `Ported` | Core transport framing and DTO ownership are present. Any remaining gap is observability, not protocol ownership. | `cpp/src/protocol` |
| `src/protocol/request_models.py` | Drive-request normalization from HTTP/WebSocket/controller intent into shared internal target representation | `cpp/include/robot/protocol/api_drive_request.hpp`, `cpp/src/protocol/api_drive_request.cpp` | `Ported` | Core normalization contract exists in C++. | `cpp/src/protocol` |
| `src/control/controller_input_mapping.py` | Trigger hysteresis, stale/disconnect handling, dead zones, controller mode classification | `cpp/include/robot/control/controller_drive_mapper.hpp`, `cpp/src/control/controller_drive_mapper.cpp` | `Ported` | Core mapping semantics migrated. Remaining gap is exported diagnostics around transitions and mode changes. | `cpp/src/control` |
| `src/control/drive_command_manager.py` | Command-source ownership, stale sequence rejection, timeout-stop guard, command stop logging | `cpp/src/control/drive_command_manager.cpp` | `Partial` | Core admission policy migrated. Logging and explicit command-source breadcrumbs did not. | `cpp/src/control` emits events; formatting stays out of control and belongs to support/service code |
| `src/control/drive_target.py`, `src/control/drive_target_store.py` | Drive target value object and persistent storage | `cpp/include/robot/control/drive_target.hpp`, `cpp/include/robot/control/drive_target_store.hpp`, `cpp/src/control/drive_target_store.cpp` | `Ported` | Core target ownership migrated. | `cpp/src/control` |
| `src/control/drivetrain_controller.py` | Closed-loop drivetrain worker, lifecycle transitions, fault latching, recover, watchdog staging, diagnostics and logs | `cpp/src/control/drivetrain_controller.cpp`, `cpp/include/robot/control/drivetrain_controller.hpp`, Pico-specific drivers under `cpp/firmware/src/hardware` and `cpp/firmware/src/platform` | `Partial` | Core state machine, timeout stop, recover, watchdog staging, and hardware ownership migrated. Lifecycle logs, state snapshots, bench summaries, and fault breadcrumbs are not yet exported in comparable form. | Motion ownership stays in `cpp/src/control`; hardware interactions in `cpp/firmware/src/hardware`; event export to runtime/service side |
| `src/control/pid_fork.py` | Deterministic PID primitive with microsecond timing and explicit integral limits | `cpp/src/control/pid.cpp`, `cpp/include/robot/control/pid.hpp` | `Ported` | Core PID primitive exists in C++. | `cpp/src/control` |
| `src/hardware/encoder_subsystem.py` | Persistent encoder owner, velocity estimator, fault state, reset/recover, diagnostics and reporting | `cpp/include/robot/hardware/drivetrain_encoder_subsystem.hpp`, `cpp/src/hardware/drivetrain_encoder_subsystem.cpp`, Pico-specific counter in `cpp/firmware/src/hardware/pico_quadrature_counter.cpp` | `Partial` | Persistent ownership and recovery model migrated. Rich diagnostics, health snapshots, and log/report surfaces are still thinner than the MicroPython runtime. | Ownership in `cpp/src/hardware`; Pico-specific counter primitive in `cpp/firmware/src/hardware` |
| `src/hardware/encoder_pio.py` | Low-level PIO quadrature counter | `cpp/firmware/src/hardware/pico_quadrature_counter.cpp`, `cpp/firmware/quadrature_counter.pio` | `Ported` | Pico-specific primitive exists in the C++ firmware tree. | `cpp/firmware/src/hardware` |
| `main.py` and `tools/pico_deploy.py` | MicroPython bootstrap and file-sync deployment workflow | UF2 build/flash workflow under `cpp/`; no repo-local helper script yet | `Deferred` | C++ runtime is flashed as a UF2 and does not use the MicroPython deploy path. The repo still lacks a polished C++ build/flash helper flow comparable in convenience to `tools/pico_deploy.py`. | Build/flash helpers belong in `tools/`; runtime behavior docs belong in `docs/` |

## Consolidated Missing Or Incomplete Migrations
### Logging and telemetry
- No C++ replacement exists yet for the MicroPython helper surface:
  `loge/logw/logi/logd/logv`, grouped logger imports, and shared formatting.
- `print()` mirroring is intentionally not part of the target design, but the
  replacement single-sink stdio logger is also not implemented yet.
- ANSI styling, level filtering, tag overrides, and tick-prefixed lines are
  missing from the C++ runtime.
- There is no separate structured telemetry helper in the C++ runtime; the
  target is one parsable log stream with stable tags and message fields.

### Boot and service lifecycle
- No C++ boot banner or reset-cause breadcrumb exists.
- No explicit C++ log is emitted when controller ingress becomes active,
  Wi‑Fi connect starts, Wi‑Fi succeeds, Wi‑Fi fails, or the server starts.
- Wi‑Fi retry/backoff policy fields exist in config but are not honored by the
  current `pico_service_core.cpp` implementation.

### Operator transport and diagnostics
- Route-level request/ack/error logging is missing in the Mongoose transport.
- WebSocket/HTTP error mapping is less complete than the MicroPython runtime.
- No C++ equivalent exists yet for sampled transport logs and recoverable route
  exception breadcrumbs.

### Controller pipeline observability
- The controller UART pipeline was migrated as control logic, but not as an
  operator-visible diagnostics surface.
- Mode transitions, stop reasons, button-change diagnostics, and structured
  controller reporting are still absent from the C++ runtime.

### Control and hardware event export
- Command manager, drivetrain controller, and encoder subsystem now own the
  right behavior classes in C++, but they do not yet expose equivalent
  higher-level events for service-side logging and diagnostics.
- Existing C++ control/hardware code should not grow formatting logic; it
  should expose event data or status snapshots that the support/service layer
  can format.

### Tooling and bench workflow
- The MicroPython deploy workflow is mature and repo-owned.
- The C++ UF2 build/flash workflow is functional but still lacks repo-local
  convenience tooling and bench-oriented documentation parity.

## Target C++ Responsibility Map
### `cpp/include/robot/support` and `cpp/src/support`
- Own the new logger facade and helper methods.
- Own formatting rules for levels, tags, timestamps, ANSI styling, and
  parseable log-line formatting.
- Must not own backend routing policy beyond writing to standard stdio.

### `cpp/firmware/src/platform`
- Own Pico runtime lifecycle logging such as boot, reset cause when available,
  Wi‑Fi connect progress, success/failure, and stdio initialization.
- Own service-side platform integrations and status surfaces.

### `cpp/firmware/src/transport`
- Own request/ack/error diagnostics for HTTP/WebSocket handling.
- Own transport-level protocol error logging and queue-full breadcrumbs.
- Must not own motion acceptance decisions.

### `cpp/src/runtime`
- Own service/control handoff surfaces and exported status snapshots.
- Provide the bounded bridge that lets service-side code observe control-owned
  events without moving decision ownership out of control.

### `cpp/src/control`
- Continue owning controller semantics, command admission, stale-input stop,
  drivetrain state, watchdog authority, and final motion acceptance.
- Expose event data needed for logging, but do not format or route logs here.

### `cpp/src/hardware` and `cpp/firmware/src/hardware`
- Continue owning encoder and motor hardware truth.
- Expose recoverable hardware fault/state information needed for diagnostics.

### `tools/` and `docs/`
- `tools/` should eventually carry convenience helpers for C++ build/flash
  workflows.
- `docs/` should remain the source of truth for migration status and bench
  behavior differences between MicroPython and C++.

## Porting Order Recommended By This Audit
1. Add the C++ support-layer logger and caller-side filters with one stdio sink.
2. Restore boot and Wi‑Fi lifecycle breadcrumbs on the service side.
3. Restore transport-side request/ack/error diagnostics.
4. Export controller-pipeline and drivetrain event/status data for logging.
5. Add C++ build/flash/bench helpers so operator workflow parity improves
   alongside runtime parity.

## Notes
- A quiet USB serial terminal after flashing the current UF2 is expected: the
  current C++ runtime initializes stdio but does not yet recreate the old
  MicroPython logging layer or its call sites.
- This audit intentionally separates core control parity from observability
  parity. The C++ tree already carries much of the control-domain migration,
  but service-side diagnostics and tooling are still materially behind the
  active MicroPython runtime.
