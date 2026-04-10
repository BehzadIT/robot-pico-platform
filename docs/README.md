# Pico Platform Documentation

This directory contains Pico-specific implementation documentation for the robot
firmware and control-side logic.

Use the parent `../docs` directory only when this repo is checked out inside the
`ai-robot` workspace. The Pico repo remains usable as an independent Git
repository.

## Documents
- `firmware-architecture.md`: Pico runtime structure and module ownership.
- `motor-control-and-encoders.md`: drivetrain and encoder implementation notes.
- `logging-and-telemetry.md`: Pico-side logging and UART telemetry behavior.
- `drivetrain-pid-pio-reliability-review.md`: detailed reliability review and
  implementation priorities for PID, PIO, timing, and fault handling.

## Parent Context
When available, read:
- `../../docs/overview/current-architecture.md`
- `../../docs/interfaces/command-protocol.md`
- `../../docs/interfaces/telemetry-protocol.md`
- `../../docs/electrical/wiring-and-pinout.md`
- `../../docs/verification/known-issues-and-open-questions.md`
