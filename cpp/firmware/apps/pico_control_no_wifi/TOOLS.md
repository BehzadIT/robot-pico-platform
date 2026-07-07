# Pico Clean App Tooling

## Purpose
This file records the host-side tools used for the clean Pico C++ feedback loop.

The clean app is `robot_pico_control_no_wifi`. It is intentionally separate from
the migrated C++ runtime and currently proves USB stdio, flash/reboot workflow,
startup logs, FreeRTOS SMP task startup, pinned service/control task ownership,
and deterministic skeleton heartbeat output.

## Tools
### `picotool`
`picotool` is Raspberry Pi's host CLI for inspecting, flashing, and rebooting
RP2040/RP2350 devices over USB.

Installed local binary:

```bash
/home/behzad/.local/bin/picotool
```

Expected version shape:

```bash
picotool version
```

The installed binary must include USB support. If `picotool version` says it was
compiled without USB support, flash and reboot commands will not work.

Useful commands:

```bash
picotool info -a
picotool reboot -f
picotool reboot -f -u
```

Meanings:
- `picotool info -a`: inspect the connected app/device.
- `picotool reboot -f`: reboot back into the normal app.
- `picotool reboot -f -u`: reboot into BOOTSEL / UF2 mode.

If permissions fail, install the bundled udev rule and reconnect the Pico:

```bash
sudo install -m 0644 cpp/build-pico2-cross/_deps/picotool-src/udev/60-picotool.rules /etc/udev/rules.d/60-picotool.rules
sudo udevadm control --reload
sudo udevadm trigger
```

## Build And Flash
Build the clean app:

```bash
python3 tools/pico_cpp_deploy.py build
```

Flash through `picotool` when USB permissions and USB-enabled `picotool` are
working:

```bash
python3 tools/pico_cpp_deploy.py flash --build
```

Flash through BOOTSEL mass storage when the Pico is mounted as `RP2350`:

```bash
cp /home/behzad/Projects/ai-robot/robot-pico-platform/cpp/build-pico2-cross/firmware/apps/pico_control_no_wifi/robot_pico_control_no_wifi.uf2 /run/media/behzad/RP2350/
sync
```

Equivalent helper command:

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
python3 tools/pico_cpp_deploy.py monitor --timeout 30
```

Important learned workflow: if boot logs matter, start `monitor` first, then
flash or reboot the Pico from another command. The helper waits for
`/dev/ttyACM0`, prints serial output into command output, and reconnects if the
Pico disappears during a reboot.

### `minicom`
`minicom` is an interactive serial terminal. Use it when manual interaction or a
persistent human-visible session is more useful than a bounded evidence capture.

```bash
minicom -D /dev/ttyACM0 -b 115200
```

Exit minicom with `Ctrl-A`, then `X`.

Expected startup output:

```text
robot_pico_control_no_wifi: boot
robot_pico_control_no_wifi: stage=rtos_skeleton
robot_pico_control_no_wifi: board=pico2_w stdio=usb picotool_reset=enabled
robot_pico_control_no_wifi: no cyw43, no network, no transport, no hardware output
robot_pico_control_no_wifi: FreeRTOS SMP skeleton with pinned service/control tasks
robot_pico_control_no_wifi: starting scheduler
robot_pico_control_no_wifi: service started core=0 count=0 uptime_ms=...
robot_pico_control_no_wifi: control started core=1 count=0 uptime_ms=...
robot_pico_control_no_wifi: control heartbeat core=1 count=0 uptime_ms=...
robot_pico_control_no_wifi: service heartbeat core=0 count=0 uptime_ms=...
```

## Current Verified State
As of this setup, `picotool info -a` can inspect the connected clean app and
reports:
- app name: `robot_pico_control_no_wifi`
- target chip: `RP2350`
- board: `pico2_w`
- feature: USB stdin/stdout

That means the command-based USB tool path is working for the clean app.
