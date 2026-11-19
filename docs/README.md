# Pico Platform Documentation

This directory contains Pico-specific implementation documentation for the robot
firmware and control-side logic.

Use the parent `../docs` directory only when this repo is checked out inside the
`ai-robot` workspace. The Pico repo remains usable as an independent Git
repository.

## Documents
- `firmware-architecture.md`: Pico runtime structure and module ownership.
- `micropython-to-cpp-migration-audit.md`: parity audit and migration gap list
  between the active MicroPython runtime and the staged Pico SDK / C++ runtime.
- `runtime-boundaries.md`: RTOS/service versus control-domain ownership rules
  for the C++ migration.
- `motor-control-and-encoders.md`: drivetrain and encoder implementation notes.
- `logging-and-telemetry.md`: Pico-side logging and UART telemetry behavior.
- `drivetrain-pid-pio-reliability-review.md`: detailed reliability review and
  implementation priorities for PID, PIO, timing, and fault handling.
- `firmware-dependencies.json`: Pico runtime dependency and local fork metadata.
- `code-documentation.md`: how to document MicroPython control, transport, and
  safety code without obscuring timing-critical behavior.

## Board baseline
- The current Pico SDK migration target is Raspberry Pi Pico 2 W.
- Use `PICO_BOARD=pico2_w` for firmware configuration unless a document
  explicitly calls out a different board.

## Agent Verification Adapter

`python3 tools/robot_adapter.py doctor --json` performs passive Pico toolchain
discovery. The default host and firmware scenarios reuse CMake, CTest, and
`pico_cpp_deploy.py`; they do not merge the MicroPython, migrated C++, and
clean no-network targets. Flash/boot and physical scenarios are separately
gated, and Pico remains the final control and safety authority.

Hardware manuals, wiring notes, component lists, and mechanical images live in
the parent `../../docs/assets` tree when this repo is checked out inside
`ai-robot`.

## Parent Context
When available, read:
- `../../docs/overview/current-architecture.md`
- `../../docs/interfaces/command-protocol.md`
- `../../docs/interfaces/telemetry-protocol.md`
- `../../docs/electrical/wiring-and-pinout.md`
- `../../docs/verification/known-issues-and-open-questions.md`
