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
