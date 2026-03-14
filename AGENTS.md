# AGENTS.md

## Purpose
This repo contains the Pico firmware and control-side logic for the robot.

## Instructions
- Before changing code here, read the shared docs root at `/home/behzad/Projects/robot-project-docs`.
- At minimum, review:
  - `/home/behzad/Projects/robot-project-docs/overview/current-architecture.md`
  - `/home/behzad/Projects/robot-project-docs/firmware/pico-firmware-architecture.md`
  - `/home/behzad/Projects/robot-project-docs/firmware/motor-control-and-encoders.md`
  - `/home/behzad/Projects/robot-project-docs/firmware/logging-and-telemetry.md`
  - `/home/behzad/Projects/robot-project-docs/electrical/wiring-and-pinout.md`
  - `/home/behzad/Projects/robot-project-docs/verification/known-issues-and-open-questions.md`
- Preserve the current architectural boundary: the Pico owns real-time control.
- Be careful around encoder pin assumptions, logging behavior, and REPL compatibility.
- When making non-trivial decisions, state the relevant best practice, design pattern, technology choice, and architecture tradeoff.
- Prefer reliability-oriented patterns for networking and control paths: explicit state machines, bounded retry/backoff, authoritative failsafes, and structured diagnostics.
- Log meaningful transport, protocol, and safety events. Malformed or invalid commands should be treated as recoverable protocol events unless safety requires closing the session.
- Code documentation matters in this project. Add clear comments or docstrings for non-obvious behavior, especially around lifecycle, protocol semantics, safety behavior, hardware ownership, fault handling, recovery, timing assumptions, and sign conventions.
- Do not add comments for obvious line-by-line mechanics; document the parts another engineer could misread and accidentally break.
- If code changes invalidate the shared docs, update the docs as part of the same work.

## Structure Pattern
- Use firmware-oriented structure, not web-app layering names.
- Prefer these parent directories and meanings:
  - repo-root `main.py` for the firmware bootstrap entrypoint.
  - `src/` for project-owned runtime code.
  - `src/app/` for bootstrapping, startup lifecycle, and top-level composition.
  - `src/control/` for drivetrain control logic, safety state machines, and closed-loop behavior.
  - `src/protocol/` for command schema, protocol validation, DTOs, acknowledgements, and error codes.
  - `src/transport/` for websocket, HTTP, request routing, and connection/session handling.
  - `src/platform/` for Pico/Wi-Fi/runtime integration details.
  - `src/hardware/` for hardware-facing drivers and pin-bound subsystems such as encoders or motor interfaces.
  - `src/support/` for logging and tightly scoped shared utilities.
  - `lib/` for deployed runtime dependency code and vendored libraries used by MicroPython on-device.
  - `settings/` for Pico-side configuration that is uploaded with the firmware.
  - `settings/config.py` for tracked non-secret runtime settings.
  - `settings/secrets.py` for local-only secrets such as Wi-Fi credentials; keep it out of git but include it in Pico uploads.
  - `tests/` for retained tests.
  - `experiments/` for bench scripts and temporary tuning work that should not sit at repo root.
- Do not use `services/` as a long-term catch-all. Split by responsibility:
  - control behavior goes to `control/`
  - hardware-facing code goes to `hardware/`
  - protocol objects go to `protocol/`
- Do not use `server/` as a broad bucket when the code is specifically transport or protocol handling.
- Keep the current architectural boundary intact while restructuring:
  - Pico remains authoritative for real-time control and safety.
  - transport/protocol code may request actions, but control and hardware ownership stay below `control/` and `hardware/`.
- For the current repo, the preferred target shape is:
  - repo-root `main.py` as a thin bootstrap entrypoint, delegating into `src/app/`.
  - `settings/` as the only root-level firmware config directory.
  - `settings/config.py` for tracked non-secret runtime configuration.
  - `settings/secrets.py` for local-only secrets that are uploaded to the Pico.
  - `src/transport/` for the websocket route handlers and session handling.
  - `src/protocol/` for request models, command parsing, protocol constants, and ack/error payload helpers.
  - `src/control/` for navigation controller, PID orchestration, drivetrain policy, and safety timing logic.
  - `src/hardware/` for encoder and hardware-bound modules.
  - `src/platform/` for Wi-Fi and MicroPython runtime-specific integration.
  - `src/support/` for logging and narrow shared helpers.
  - `lib/` for vendored libraries such as Microdot, websocket helpers, and third-party runtime packages.
  - `docs/` for reference material, including future hardware notes/assets.
  - `experiments/` for RPM tests, balancing scripts, and temporary bench programs.
- When adding a file, place it by hardware/control/transport responsibility first, and only use broad shared folders when reuse is already real.
