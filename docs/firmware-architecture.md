# Pico Firmware Architecture

## Purpose
Describe the current Raspberry Pi Pico firmware structure and runtime responsibilities.

## Current status
- `Implemented`: MicroPython application in `robot-pico-platform` owns Wi-Fi connection, control routes, motor control, and logging.

## Entry flow
- repo-root `main.py` delegates into `src/app/`.
- The runtime enables UART print mirroring.
- It creates a Microdot app.
- It registers routes.
- It reads tracked runtime settings from `settings/config.py`.
- It reads local Wi-Fi credentials from `settings/secrets.py`.
- It connects to Wi-Fi.
- It starts the HTTP server on port `80`.

## Key modules
- `src/transport/mobile_app_transport.py`: mobile-app-facing HTTP and WebSocket control endpoints
- `src/control/drive_command_manager.py`: drive command admission, source ownership, and drive target handoff
- `src/control/drivetrain_controller.py`: drive loop, PWM output, encoder reads
- `src/support/logger.py`: REPL-safe logging implementation
- `src/platform/wifi_connection.py`: station-mode Wi-Fi join logic

## Root layout
- `main.py`: Pico firmware bootstrap
- `src/`: project-owned runtime code
- `lib/`: runtime dependencies deployed to the Pico
- `settings/`: Pico-side configuration package
- `docs/`: reference material
- `experiments/`: non-production tuning and exploratory scripts

## Notes
The Pico firmware is currently both the real-time controller and the live control server. That is a current implementation fact even if future architecture changes move command routing elsewhere.
The shared upload workflow targets the real firmware paths (`main.py`, `src/`, `lib/`, `settings/`) via `tools/pico_deploy.py` and root-relative exclusions from `.picodeployignore`, rather than a mirrored deploy tree.
Drivetrain watchdog settings are owned by `settings/config.py` and are rolled
out in stages. The current safe pattern is to defer watchdog arming until after
the first clean stop cycle instead of arming immediately at startup.

## Sources
- Code: [main.py](../main.py)
- Code: [mobile_app_transport.py](../src/transport/mobile_app_transport.py)
- Code: [drive_command_manager.py](../src/control/drive_command_manager.py)
- Code: [drivetrain_controller.py](../src/control/drivetrain_controller.py)
