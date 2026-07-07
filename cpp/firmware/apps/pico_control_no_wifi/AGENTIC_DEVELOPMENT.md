# Clean App Agentic Development Loop

## Purpose
This is the lightweight working loop for agents changing the fresh Pico clean
app. It is a local patch to make feedback faster while the larger project
workflow is still deferred.

The parent workflow guidance lives at:

```text
../../../../../docs/operations/agentic-development-workflow.md
```

## Boundary
This app is the fresh no-network Pico starting point. Do not pull behavior from
the existing migrated C++ runtime unless the task explicitly asks for it.

Current app responsibilities:
- USB stdio startup
- picotool-compatible USB reset support
- startup logs
- FreeRTOS SMP scheduler startup
- pinned service task on core 0
- pinned control skeleton task on core 1
- service/control heartbeat logs
- fast build/flash/observe feedback loop

Current non-responsibilities:
- CYW43, Wi-Fi, Bluetooth, HTTP, or WebSocket
- ESP32 command transport
- UART receive, frame parsing, or command DTO handoff
- motor output
- encoder ownership
- watchdog-controlled motion
- safety or command-admission policy

## Required Loop
For every code or build change in this directory:

1. Build it yourself.
2. Run any automatic check that fits the change.
3. If Pico hardware is connected, flash and observe it yourself before handing
   work back.
4. Give the user the exact command to run only when the board is absent,
   inaccessible, or needs physical intervention.
5. State the expected signal, such as a startup line, heartbeat, reboot mode, or
   `picotool info` field.

Default build command from `robot-pico-platform/`:

```bash
python3 tools/pico_cpp_deploy.py build
```

Useful automatic checks:

```bash
python3 -m py_compile tools/pico_cpp_deploy.py
ctest --test-dir cpp/build-host --output-on-failure
picotool info -a
```

Use only checks relevant to the change. For example, a pure docs edit does not
need a firmware rebuild.

## Agent-Run Hardware Loop
When the Pico is connected, the agent should run the feedback loop end to end.
The clean app supports this because it enables picotool-compatible USB reset.

For simple true/false checks, do not hand observation to the user. Build, flash,
capture logs, and report the concrete result. Ask the user to observe only when
the result depends on physical motion, safety posture, wiring, sound, heat,
mechanical behavior, or another subjective/bench-side condition.

When boot logs matter, start log capture first, then reboot or flash. The
project helper monitor waits for `/dev/ttyACM0` and reconnects after USB
disappears during a reboot, so it is better than starting capture after the
Pico has already booted.

Preferred command-based flow from `robot-pico-platform/` in two agent sessions:

```bash
python3 tools/pico_cpp_deploy.py monitor --timeout 30
```

Then, while the monitor is running:

```bash
python3 tools/pico_cpp_deploy.py flash --build
```

If the board is already in BOOTSEL / mass-storage mode, flash by copying the
UF2 while the monitor is already running:

UF2 copy flow:

```bash
python3 tools/pico_cpp_deploy.py flash-msd --build
```

## Serial Observation Tool Guidance
For Pico clean-app hardware verification, use the repo helper `monitor` for
agent-run evidence capture and `minicom` for human interactive observation.

### `python3 tools/pico_cpp_deploy.py monitor`
This is a lightweight Python/pyserial-based serial reader in the repo helper. It
opens the Pico serial port, reads bytes, prints them to the agent terminal, and
can exit automatically with `--timeout`.

Use it when:
- an agent needs non-interactive proof of boot logs or heartbeat logs;
- the result should be captured directly in command output;
- the check is mostly true/false, such as "did the boot banner appear?" or "are
  service/control heartbeats present?";
- the agent will report exact observed lines back to the user.

Example:

```bash
python3 tools/pico_cpp_deploy.py monitor --timeout 12
```

Important learned workflow: if boot logs matter, start `monitor` first, then
flash or reboot the Pico from another command. The helper now waits for
`/dev/ttyACM0` and reconnects across USB disappearance, so it can capture the
fresh boot banner after flashing or app-mode reboot.

### `minicom`
`minicom` is an interactive serial terminal. Use it when manual interaction or a
persistent human-visible session is more useful than a bounded evidence capture.

```bash
minicom -D /dev/ttyACM0 -b 115200
```

Exit minicom with `Ctrl-A`, then `X`.

If the Pico is in application mode, the agent may use the helper to reboot it:

```bash
python3 tools/pico_cpp_deploy.py reboot --mode bootsel
python3 tools/pico_cpp_deploy.py reboot --mode app
```

Hand the task back to the user only when:
- no Pico USB device or mass-storage mount is visible;
- USB permissions prevent `picotool` or serial access;
- the board fails to re-enumerate after flashing;
- the serial stream does not show boot output after a reasonable retry.

Expected proof-of-life:

```text
robot_pico_control_no_wifi: boot
robot_pico_control_no_wifi: stage=rtos_skeleton
robot_pico_control_no_wifi: service started core=0 count=0 uptime_ms=...
robot_pico_control_no_wifi: control started core=1 count=0 uptime_ms=...
robot_pico_control_no_wifi: control heartbeat core=1 count=0 uptime_ms=...
robot_pico_control_no_wifi: service heartbeat core=0 count=0 uptime_ms=...
```

## Handoff
Every handoff should say:
- what changed;
- what was built or checked by the agent;
- what was flashed and observed, or why hardware intervention was required;
- what output confirms success;
- what was not verified.
