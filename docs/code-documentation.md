# Pico Code Documentation Guide

## Purpose
Document the Pico firmware so control behavior, safety policy, timing
assumptions, and hardware ownership stay readable and hard to misinterpret.

Audience:
- firmware engineers changing control, transport, or hardware-facing code;
- integrators tracing safety behavior across Pico, ESP32, and Android; and
- future maintainers validating timeout, recovery, and control-loop behavior on
  hardware.

This guide now covers both the current MicroPython runtime and the staged C++
migration tree under `cpp/`.

The active migration target is the Raspberry Pi Pico 2 W Pico SDK runtime.
Documentation should treat the `cpp/` tree as durable project code, not as a
throwaway experiment.

## Where Documentation Belongs
- Source docs: contracts and non-obvious timing/safety behavior in MicroPython
  modules.
- Markdown docs: runtime structure, multi-file drivetrain/protocol behavior,
  reliability notes, and verification gaps.
- Parent shared docs: system-wide command semantics, hardware conflicts, and
  architecture.

## What To Document
- public modules, classes, and functions;
- methods that start, stop, recover, validate, or otherwise affect control or
  hardware behavior;
- timing assumptions;
- stale-input and disconnect behavior;
- stop/fault/recover transitions;
- watchdog intent;
- sign conventions and threshold rationale.

For C++ code, also document:
- whether a class belongs to the RTOS/service side or the control domain;
- what queue/mailbox or callback boundary it is allowed to cross; and
- whether the code may block, allocate, or wait on I/O.

Use docstrings for contracts and inline comments for `why`.

## What Not To Document
- line-by-line narration in hot loops;
- obvious Python mechanics already clear from names; and
- comments that imply ESP32 or Android owns final control authority.

Keep ownership explanations in module/class docstrings, not scattered
everywhere.

## Python / MicroPython Style
- Follow PEP 257-style module, class, and function docstrings.
- Start with a short summary sentence.
- Expand only when behavior, restrictions, or side effects matter.
- Use inline comments only for timing, safety, recovery, ownership, and
  control-policy rationale.

## C++ Style
- Use Doxygen-style comments on public headers and exported types.
- Put ownership rules on the type that owns the behavior, not scattered across
  call sites.
- Document `why` for watchdog feed points, stale-input transitions, queue-drop
  policy, RTOS/service boundaries, and control-loop cadence.
- Prefer comments that name the boundary explicitly:
  - `RTOS/service side`
  - `control domain`
  - `transport/control boundary`

## Robotics Rules
- Pico owns controller-to-drive mapping, command freshness, stop/fault/recover,
  drivetrain control, and timing-sensitive hardware interaction.
- RTOS/service code may receive, parse, and buffer input, but it must not own
  motion acceptance or stale-input stop policy.
- Document degraded and recovery behavior explicitly:
  - stale command handling
  - disconnect behavior
  - fault latching
  - recovery trigger and completion rules
- Document timing and hardware assumptions:
  - control cadence
  - polling periods
  - heartbeat freshness windows
  - watchdog behavior
  - encoder assumptions
- If thresholds, timeouts, heartbeat handling, control mapping, watchdog
  behavior, or hardware assumptions change, update Markdown docs in the same
  change.
- If code moves across the RTOS/service boundary or the control boundary,
  update [runtime-boundaries.md](./runtime-boundaries.md) in the same change.

## Repo Examples
- [drivetrain_controller.hpp](../cpp/include/robot/control/drivetrain_controller.hpp)
  shows the adapted C++ ownership comments for watchdog staging, reversal
  handling, and deterministic control ownership.
- [drivetrain_encoder_subsystem.hpp](../cpp/include/robot/hardware/drivetrain_encoder_subsystem.hpp)
  shows how to retain MicroPython rationale comments while moving hardware
  ownership into C++.
- [drivetrain_controller.py](../src/control/drivetrain_controller.py)
  shows good class-level ownership and good `why` comments for timing and
  safety behavior.
- [joystick_transport.py](../src/protocol/joystick_transport.py)
  shows the right protocol-boundary docstring: parsing and CRC ownership, not
  robot meaning.
- [controller_input_mapping.py](../src/control/controller_input_mapping.py)
  is the right place to document hysteresis, dead zones, and neutral-safety
  rationale.
- [runtime-boundaries.md](./runtime-boundaries.md) is the source of truth for
  RTOS/service versus control placement during the C++ migration.

## Review Checklist
- Public modules/classes/functions have docstrings when they are part of the
  firmware contract.
- Inline comments explain timing, safety, recovery, or ownership rules.
- Comments explain `why`, not obvious Python syntax.
- Thresholds and timeouts have rationale where a maintainer could “simplify”
  them incorrectly.
- Control, freshness, or hardware-assumption changes update Markdown docs too.
- Migration progress that changes the source of truth from MicroPython to C++
  updates the durable docs in `docs/` in the same change.

## References
- PEP 257:
  https://peps.python.org/pep-0257/
