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
- `src/transport/navigation.py`: HTTP and WebSocket control endpoints
- `src/control/robot_drive_controller.py`: navigation target state and stop handling
- `src/control/robot_cross_coupled_pid.py`: drive loop, PWM output, encoder reads
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
The shared upload workflow should target the real firmware paths (`main.py`, `src/`, `lib/`, `settings/`) rather than a mirrored deploy tree.
Drivetrain watchdog settings are owned by `settings/config.py` and are rolled
out in stages. The current safe pattern is to defer watchdog arming until after
the first clean stop cycle instead of arming immediately at startup.

## Sources
- Code: [main.py](../main.py)
- Code: [navigation.py](../src/transport/navigation.py)
- Code: [robot_drive_controller.py](../src/control/robot_drive_controller.py)
- Code: [robot_cross_coupled_pid.py](../src/control/robot_cross_coupled_pid.py)
