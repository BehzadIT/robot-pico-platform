# robot-pico-platform

## Pico Deploy
Use the tracked CLI deploy script:

```bash
python3 tools/pico_deploy.py
```

Optional port override:

```bash
python3 tools/pico_deploy.py --port /dev/ttyACM0
```

Deploy behavior:
- Managed deploy roots are `main.py`, `src/`, `lib/`, and `settings/`.
- Default port is `/dev/ttyACM0`; use `--port` only when needed.
- Exclusions come from `.picodeployignore`.
- The script kills stale local port holders before deploy.
- It compares staged files against Pico-side SHA-256 hashes, uploads only changed files, deletes stale managed paths, then opens a live serial session and triggers the soft reboot from that attached session.
- Terminal output is intentionally concise: high-signal deploy states, then only the files that were actually updated, then live Pico boot/REPL output.
- If deploy fails, the script can interactively try recovery in order: soft reset, hard reset, clear Pico data, then retry.
- Exit the attached session with `Ctrl-]` or `Ctrl-X`.

Uploaded firmware paths:
- `main.py`
- `src/`
- `lib/`
- `settings/`

Local-only device file:
- `settings/secrets.py`

`settings/secrets.py` must exist on your machine before upload unless you explicitly exclude it in `.picodeployignore`. It must define:

```python
WIFI_CREDENTIALS = {
    "ssid": "your-wifi-ssid",
    "password": "your-wifi-password",
}
```

`settings/config.py` is versioned and contains non-secret Wi-Fi/runtime settings such as timeouts and retry behavior.

Drivetrain watchdog settings live in `settings/config.py`.
For rationale and stability notes, see the shared firmware docs:
- `../docs/firmware/logging-and-telemetry.md`
- `../docs/firmware/pico-firmware-architecture.md`

Root directory purpose:
- `main.py`: Pico firmware bootstrap
- `src/`: project-owned runtime code
- `lib/`: MicroPython runtime dependencies
- `settings/`: Pico configuration package
- `docs/`: reference material
- `experiments/`: non-production exploratory scripts

Pico-specific implementation documentation starts at `docs/README.md`.

## Dependency Workflow
This repo uses a vendored dependency model for Pico runtime libraries.

Rules:
- Keep third-party runtime dependencies in `lib/`.
- Keep project-owned runtime code in `src/`; do not place local transport/helpers in `lib/`.
- Track vendored dependencies in git and record them in `docs/firmware-dependencies.json`.
- Do not rely on on-device `mip`/`upip` installs as the primary workflow.
- Keep deployable `lib/` content clean: no editor metadata, `__pycache__`, or `.dist-info` packaging folders.
- Keep Microdot pinned to an explicit upstream release. The current target is the official `2.5.1` package layout, including `microdot.websocket`.

Normal update flow:
1. Pick a MicroPython-compatible upstream release or commit.
2. Replace or update the vendored files under `lib/`.
3. Update `docs/firmware-dependencies.json` with the pinned version, source URL, and local paths.
4. Remove packaging/editor artifacts that should not be uploaded to the Pico.
5. Run local smoke checks and verify imports.
6. Upload with `python3 tools/pico_deploy.py`.
7. Verify boot, Wi-Fi join, transport startup, and logging behavior on-device.

Microdot note:
- Core app imports use `from microdot import Microdot`.
- WebSocket support comes from the official `microdot.websocket` extension.

PID note:
- `src/control/pid_fork.py` is a project-owned local fork derived from `gastmaier/micropython-simple-pid`, not a vendored runtime library.
- The drivetrain relies on the current fork behavior, including explicit microsecond timing and independent `integral_limits`.
- Treat any PID library update as a control-path change that needs regression coverage and bench verification, not as a routine dependency bump.
