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
- ESP32 remains controller/log transport side only.
- Android remains operator input only.

## Current migration status
- The staged C++ runtime under `cpp/` now contains:
  - bounded inter-core mailboxes
  - control-domain command admission
  - controller UART ingest and mapping on the control core
  - a persistent encoder owner
  - a drivetrain state machine and PID controller
  - Pico SDK firmware scaffolding for dual-core FreeRTOS bring-up
- The active board target for this migration is Raspberry Pi Pico 2 W
  (`PICO_BOARD=pico2_w`).
- MicroPython remains in-tree until the Pico SDK firmware passes hardware and
  bench parity validation.

## Core ownership on Pico 2 W
- Core 1 owns the deterministic control domain:
  - controller UART ingest and mapping
  - drive-command admission and freshness policy
  - encoder truth and drivetrain state
  - final motor output and safety stop decisions
  - fixed-cadence control-loop updates
- Core 0 owns the RTOS/service domain:
  - Wi-Fi join, reconnect, and network stack service
  - HTTP/WebSocket session handling
  - UART byte ingress and framed transport parsing
  - protocol schema parsing and protocol-level error replies
  - logging drain, diagnostics, and bounded buffering
- Both cores are managed by FreeRTOS. The boundary is task ownership and core
  affinity, not "one core outside RTOS" and one core inside it.

## RTOS/service side
Good RTOS/service responsibilities:
- Wi-Fi join, reconnect, and network stack service
- HTTP/WebSocket session handling
- UART byte ingress and framed transport parsing
- protocol schema parsing and protocol-level error replies
- logging drain
- diagnostics/status endpoints
- bounded buffering between I/O producers and control consumers

RTOS/service code may:
- wait on sockets, UART, and network state
- package parsed transport data into fixed-size DTOs
- acknowledge protocol success/failure
- export diagnostics and logs

RTOS/service code must not:
- decide whether stale input should stop the robot
- own command-source arbitration
- own drivetrain state transitions
- write PWM/DIR outputs
- feed the watchdog based on transport-only progress

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

## Narrow handoff
The preferred seam is:
- RTOS/service side produces `JoystickSnapshot` or parsed operator command DTOs.
- Control code converts those into robot meaning and motion decisions.

Do not split the joystick teleop pipeline casually. For the first migration,
keep UART ingest, frame decode, snapshot freshness, and controller-intent
production unified until the handoff into command admission.

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
