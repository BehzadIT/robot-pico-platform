# Runtime Boundaries

## Purpose
Define which Pico firmware responsibilities may live on the RTOS/service side
 during the C++ migration, which responsibilities must remain in the
 deterministic control domain, and what handoff is allowed between them.

This is a long-lived architecture document. It should track the real migration
state as code lands, not just the intended end state.

## Core rule
- Transport packages data from the outside world.
- Control decides whether the robot may move.
- Hardware-facing control owns final actuator output and safety state.

This rule preserves the current system boundary:
- Pico remains the real-time control and safety authority.
- ESP32 remains transport side only, including the target wireless gateway.
- Android remains operator input only.

## Current migration status
- The staged C++ runtime under `cpp/` now contains:
  - bounded inter-core mailboxes
  - control-domain command admission
  - current/staging controller UART ingest and mapping on the control core
  - a persistent encoder owner
  - a drivetrain state machine and PID controller
  - Pico SDK firmware scaffolding for dual-core FreeRTOS bring-up
- The active board target for this migration is Raspberry Pi Pico 2 W
  (`PICO_BOARD=pico2_w`).
- The target clean-start C++ control firmware should not require Pico
  Wi-Fi/CYW43 initialization. ESP32 should own the robot wireless gateway while
  Pico boots into USB logging, UART transport, and control ownership.
- For the clean-start target, the Raspberry Pi Pico 2 W is still the board
  target but not a network participant. Do not put Bluetooth, HTTP, WebSocket,
  or Wi-Fi gateway responsibilities on the Pico.
- MicroPython remains in-tree until the Pico SDK firmware passes hardware and
  bench parity validation.

## Core ownership on Pico 2 W
- Core 1 owns the deterministic control domain:
  - validated controller/operator intent mapping
  - drive-command admission and freshness policy
  - encoder truth and drivetrain state
  - final motor output and safety stop decisions
  - fixed-cadence control-loop updates
- Core 0 owns the RTOS/service domain:
  - UART transport with the ESP32 wireless gateway
  - UART byte ingress and framed transport parsing
  - protocol schema parsing and protocol-level error replies
  - logging drain, diagnostics, and bounded buffering
- In the current Pico-network firmware path, core 0 may also own Wi-Fi join,
  reconnect, network stack service, and HTTP/WebSocket session handling. Those
  responsibilities are not required for the target clean-start C++ control
  firmware and should not be added to that target.
- Both cores are managed by FreeRTOS. The boundary is task ownership and core
  affinity, not "one core outside RTOS" and one core inside it.
- Use pinned tasks for the clean-start firmware. Do not rely on unconstrained
  SMP scheduling for timing-sensitive control.

For the clean-start ESP32-gateway architecture, move ESP32 UART byte ingress and
frame parsing to core 0. Core 1 should consume fixed-size DTOs from the bounded
handoff during the control tick; it should not poll UART bytes directly.

## RTOS/service side
Good RTOS/service responsibilities:
- UART transport with the ESP32 wireless gateway
- UART byte ingress and framed transport parsing
- protocol schema parsing and protocol-level error replies
- logging drain
- diagnostics/status endpoints
- bounded buffering between I/O producers and control consumers
- short critical sections around shared queues, status snapshots, UART
  ownership, or logging sinks

Current Pico-network firmware may additionally place these responsibilities on
the RTOS/service side:
- Wi-Fi join, reconnect, and network stack service
- HTTP/WebSocket session handling

RTOS/service code may:
- wait on sockets, UART, and network state
- package parsed transport data into fixed-size DTOs
- acknowledge protocol success/failure
- export diagnostics and logs
- drop best-effort logs or status under pressure

RTOS/service code must not:
- decide whether stale input should stop the robot
- own command-source arbitration
- own drivetrain state transitions
- write PWM/DIR outputs
- feed the watchdog based on transport-only progress
- hold locks while sleeping, logging large messages, parsing large input, or
  waiting on I/O
- delay the core 1 control tick for logs, diagnostics, or UART bursts

## Non-RTOS control domain
Control-domain responsibilities:
- controller snapshot meaning and teleop mapping
- command admission and source ownership policy
- stale-input and disconnect stop behavior
- drive target ownership and freshness rules
- drivetrain state machine and fixed-cadence update loop
- encoder ownership and motion-feedback truth
- PID state and final actuator commands
- fault latching, recover semantics, and watchdog progress

Control-domain code should prefer:
- explicit state machines
- fixed-cadence loops
- preallocated state and bounded mailboxes
- transition-based logs instead of per-tick logs
- reading latest validated DTOs during the control tick instead of polling raw
  transport bytes

## Narrow handoff
The preferred seam is:
- RTOS/service side produces `JoystickSnapshot` or parsed operator command DTOs.
- Control code converts those into robot meaning and motion decisions.
- The handoff is bounded, fixed-size, and non-blocking relative to the control
  tick.

Do not split the joystick teleop pipeline casually in the current migration
path. For the clean-start ESP32-gateway target, the intended split is:
- core 0 owns UART ingest, frame decode, and transport validation;
- core 1 owns snapshot/intent meaning, freshness, command admission, and motion
  decisions.

## Current module mapping
- C++ migration tree:
  - `cpp/src/runtime/service_core_runtime.cpp`: RTOS/service-side request queueing
  - `cpp/src/runtime/control_core_runtime.cpp`: control-core handoff, command
    admission ownership, and drivetrain-runtime composition
  - `cpp/src/control/drivetrain_controller.cpp`: authoritative control owner
  - `cpp/src/hardware/drivetrain_encoder_subsystem.cpp`: control/hardware truth
  - `cpp/firmware/src/app/pico_firmware_app.cpp`: FreeRTOS bootstrap and core 1
    control-task launch
  - `cpp/firmware/src/platform/pico_service_core.cpp`: core 0 FreeRTOS service
    bring-up
  - `cpp/firmware/src/transport/mongoose_http_server.cpp`: HTTP/WebSocket service side
  - `cpp/firmware/src/hardware/pico_quadrature_counter.cpp`: Pico SDK PIO hardware primitive
- `src/app/runtime.py`: startup composition only; no drivetrain policy
- `src/platform/wifi_connection.py`: RTOS/service side
- `src/transport/mobile_app_transport.py`: RTOS/service side
- `src/protocol/request_models.py`: transport/control boundary normalization
- `src/protocol/joystick_transport.py`: RTOS/service side framing/CRC only
- `src/protocol/joystick_snapshot.py`: neutral transport DTO
- `src/transport/joystick_uart_receiver.py`: keep unified in first migration;
  split only at parsed-intent/control-admission seam
- `src/control/controller_input_mapping.py`: control domain
- `src/control/drive_command_manager.py`: control domain
- `src/control/drive_target_store.py`: control domain
- `src/control/drive_target.py`: control domain value object
- `src/control/drivetrain_controller.py`: authoritative control owner
- `src/control/pid_fork.py`: deterministic control primitive
- `src/hardware/encoder_subsystem.py`: control/hardware truth
- `src/hardware/encoder_pio.py`: low-level hardware primitive
- `src/support/logger.py`: service-side drain candidate; must stay lossy and
  non-blocking relative to control

## Review test
Use this quick test when placing code:
- If it mainly waits on I/O or packages external data, it likely belongs on the
  RTOS/service side.
- If it decides whether motion is allowed, it belongs in control.
- If it owns actuator timing or motion-feedback truth, it belongs in control.
