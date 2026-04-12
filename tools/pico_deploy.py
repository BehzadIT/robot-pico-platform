#!/usr/bin/env python3
"""Deploy managed firmware roots to a MicroPython Pico with mpremote."""

from __future__ import annotations

import argparse
import fnmatch
import json
import os
import signal
import select
import serial
import shutil
import subprocess
import sys
import tempfile
import termios
import time
import tty
from pathlib import Path
from typing import Iterable, Sequence


MANAGED_ROOTS = ("main.py", "src", "lib", "settings")
MPREMOTE_TIMEOUT_SECONDS = 15
DEFAULT_PORT = "/dev/ttyACM0"


class DeployError(RuntimeError):
    """Raised when deployment cannot continue safely."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Deploy the managed Pico firmware roots, soft reset the device, "
            "then attach to live output."
        )
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"mpremote device selector. Defaults to {DEFAULT_PORT}.",
    )
    parser.add_argument(
        "--config",
        default=".picodeployignore",
        help="Root-relative ignore file. Defaults to .picodeployignore.",
    )
    return parser.parse_args()


def project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def status(message: str) -> None:
    print(f"[pico-deploy] {message}", flush=True)


def fail(message: str) -> "DeployError":
    raise DeployError(message)


def emit_state(message: str) -> None:
    sys.stdout.write("\r")
    sys.stdout.write(f"[pico-deploy] {message}\n")
    sys.stdout.flush()


def prompt_yes_no(message: str) -> bool:
    try:
        answer = input(f"{message} [y/N]: ").strip().lower()
    except EOFError:
        return False
    return answer in {"y", "yes"}


def normalize_path(path: str) -> str:
    return path.replace("\\", "/").strip("/")


def load_patterns(config_path: Path, root: Path) -> list[str]:
    if not config_path.is_absolute():
        config_path = root / config_path
    if not config_path.exists():
        fail(f"Ignore config not found: {config_path}")

    patterns: list[str] = []
    for raw_line in config_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        patterns.append(normalize_path(line))
    return patterns


def matches_pattern(rel_path: str, is_dir: bool, pattern: str) -> bool:
    rel = normalize_path(rel_path)
    pat = normalize_path(pattern)

    if not pat:
        return False

    if fnmatch.fnmatchcase(rel, pat):
        return True

    if pat.endswith("/**"):
        base = pat[:-3].rstrip("/")
        return rel == base or rel.startswith(f"{base}/")

    if is_dir and pat.endswith("/"):
        base = pat.rstrip("/")
        return rel == base or rel.startswith(f"{base}/")

    return False


def is_ignored(rel_path: str, is_dir: bool, patterns: Sequence[str]) -> bool:
    rel = normalize_path(rel_path)
    return any(matches_pattern(rel, is_dir, pattern) for pattern in patterns)


def ensure_managed_roots(root: Path) -> None:
    missing = [name for name in MANAGED_ROOTS if not (root / name).exists()]
    if missing:
        fail(f"Missing managed deploy roots: {', '.join(missing)}")


def iter_files(root: Path, patterns: Sequence[str]) -> Iterable[tuple[Path, Path]]:
    for managed_name in MANAGED_ROOTS:
        source = root / managed_name
        rel_source = Path(managed_name)

        if source.is_file():
            if not is_ignored(managed_name, False, patterns):
                yield source, rel_source
            continue

        for current_root, dir_names, file_names in os.walk(source):
            current_path = Path(current_root)

            kept_dirs: list[str] = []
            for dir_name in sorted(dir_names):
                dir_path = current_path / dir_name
                rel_dir = dir_path.relative_to(root).as_posix()
                if not is_ignored(rel_dir, True, patterns):
                    kept_dirs.append(dir_name)
            dir_names[:] = kept_dirs

            for file_name in sorted(file_names):
                file_path = current_path / file_name
                rel_file = file_path.relative_to(root).as_posix()
                if is_ignored(rel_file, False, patterns):
                    continue
                yield file_path, file_path.relative_to(root)


def stage_payload(root: Path, patterns: Sequence[str]) -> Path:
    ensure_managed_roots(root)

    secrets = root / "settings" / "secrets.py"
    if not secrets.exists() and not is_ignored("settings/secrets.py", False, patterns):
        fail(
            "Missing required local file settings/secrets.py. "
            "Create it locally or exclude it in .picodeployignore."
        )

    temp_dir = Path(tempfile.mkdtemp(prefix="pico-deploy-"))
    stage_root = temp_dir / "payload"
    stage_root.mkdir()

    copied = 0
    for source, rel_path in iter_files(root, patterns):
        destination = stage_root / rel_path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        copied += 1

    if copied == 0:
        fail("Staged payload is empty after applying exclusions.")

    return stage_root


def find_local_port_holders(port: str) -> list[int]:
    if not port.startswith("/"):
        return []

    command = ["fuser", port]
    result = subprocess.run(command, text=True, capture_output=True, check=False)
    if result.returncode not in (0, 1):
        return []

    pids: list[int] = []
    for token in (result.stdout or "").split():
        try:
            pids.append(int(token))
        except ValueError:
            continue
    return pids


def terminate_local_port_holders(port: str) -> None:
    current_pid = os.getpid()
    holders = [pid for pid in find_local_port_holders(port) if pid != current_pid]
    if not holders:
        return

    status(f"port cleanup: stopping {len(holders)} existing process(es) using {port}")
    for pid in holders:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            continue

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        remaining = [pid for pid in find_local_port_holders(port) if pid != current_pid]
        if not remaining:
            return
        time.sleep(0.1)

    remaining = [pid for pid in find_local_port_holders(port) if pid != current_pid]
    for pid in remaining:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            continue

    time.sleep(0.1)


def ensure_port_ready(port: str) -> None:
    if port.startswith("/") and os.path.exists(port):
        terminate_local_port_holders(port)
        status(f"connecting: {port}")
        return
    if port.startswith("/"):
        fail(f"Serial port not found: {port}")


def run_mpremote(
    args: Sequence[str],
    *,
    port: str,
    check: bool = True,
    capture: bool = False,
) -> subprocess.CompletedProcess[str]:
    command = ["mpremote", "connect", port, *args]
    return subprocess.run(
        command,
        check=check,
        text=True,
        capture_output=capture,
        timeout=MPREMOTE_TIMEOUT_SECONDS,
    )


def remove_remote_path(remote_path: str, port: str) -> None:
    command = ["mpremote", "connect", port, "fs", "rm"]
    if remote_path != ":main.py":
        command.append("-r")
    command.append(remote_path)

    status("delete: " + remote_path)
    result = subprocess.run(
        command,
        text=True,
        capture_output=True,
        timeout=MPREMOTE_TIMEOUT_SECONDS,
    )
    if result.returncode == 0:
        return

    stderr = (result.stderr or "").strip()
    stdout = (result.stdout or "").strip()
    combined = "\n".join(part for part in (stdout, stderr) if part)
    missing_markers = ("No such file", "ENOENT", "does not exist")
    if any(marker in combined for marker in missing_markers):
        status(f"remote path missing, skipping: {remote_path}")
        return

    raise subprocess.CalledProcessError(
        result.returncode,
        command,
        output=result.stdout,
        stderr=result.stderr,
    )


def local_manifest(stage_root: Path) -> tuple[set[str], set[str]]:
    files: set[str] = set()
    dirs: set[str] = set()

    for path in sorted(stage_root.rglob("*")):
        rel = path.relative_to(stage_root).as_posix()
        if path.is_dir():
            dirs.add(rel)
        else:
            files.add(rel)

    return files, dirs


def remote_manifest(port: str) -> tuple[set[str], set[str]]:
    script = """
import json
import os

ROOTS = ("main.py", "src", "lib", "settings")
files = []
dirs = []

def is_dir(path):
    try:
        os.listdir(path)
        return True
    except OSError:
        return False

def walk(path):
    if is_dir(path):
        dirs.append(path)
        for name in os.listdir(path):
            child = path + "/" + name if path else name
            walk(child)
    else:
        try:
            os.stat(path)
        except OSError:
            return
        files.append(path)

for root in ROOTS:
    walk(root)

print(json.dumps({"files": sorted(files), "dirs": sorted(dirs)}))
""".strip()

    result = run_mpremote(["exec", script], port=port, capture=True)
    payload = json.loads((result.stdout or "").strip() or "{}")
    return set(payload.get("files", [])), set(payload.get("dirs", []))


def remote_root_entries(port: str) -> list[str]:
    result = run_mpremote(
        ["exec", "import json, os; print(json.dumps(sorted(os.listdir('/'))))"],
        port=port,
        capture=True,
    )
    return json.loads((result.stdout or "").strip() or "[]")


def clear_remote_root(port: str) -> None:
    entries = remote_root_entries(port)
    if not entries:
        status("recovery: remote root already empty")
        return

    status(f"recovery: clearing {len(entries)} remote root entries")
    for entry in sorted(entries, reverse=True):
        remove_remote_path(":" + entry, port)


def hard_reset(port: str) -> None:
    run_mpremote(["reset"], port=port)


def attach_repl(port: str) -> int:
    if not sys.stdin.isatty():
        emit_state("success")
        emit_state("REPL attach requires a TTY on stdin.")
        return 0

    serial_port = serial.Serial(port, baudrate=115200, timeout=0.05, write_timeout=1)
    original_termios = termios.tcgetattr(sys.stdin.fileno())

    try:
        tty.setraw(sys.stdin.fileno())
        try:
            serial_port.reset_input_buffer()
        except Exception:
            pass

        emit_state("syncing files was successful")
        emit_state("REPL output (Ctrl-] or Ctrl-X to exit) ...")

        # Reset from the attached session so early boot logs stay visible.
        serial_port.write(b"\x04")
        serial_port.flush()

        while True:
            readable, _, _ = select.select([sys.stdin, serial_port], [], [], 0.05)

            if serial_port in readable:
                data = serial_port.read(serial_port.in_waiting or 1)
                if data:
                    sys.stdout.buffer.write(data)
                    sys.stdout.buffer.flush()

            if sys.stdin in readable:
                chars = os.read(sys.stdin.fileno(), 1)
                if not chars:
                    continue
                if chars in (b"\x1d", b"\x18"):
                    emit_state("exit")
                    return 0
                serial_port.write(chars)
                serial_port.flush()
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, original_termios)
        serial_port.close()


def deploy_once(stage_root: Path, port: str) -> None:
    status("syncing files...")
    remote_files, remote_dirs = remote_manifest(port)
    local_files, local_dirs = local_manifest(stage_root)

    stale_files = sorted(remote_files - local_files, reverse=True)
    stale_dirs = sorted(remote_dirs - local_dirs, reverse=True)

    for rel_path in stale_files:
        remove_remote_path(":" + rel_path, port)
    for rel_path in stale_dirs:
        remove_remote_path(":" + rel_path, port)

    updated_files: list[str] = []
    for rel_path in sorted(local_files):
        result = run_mpremote(
            ["fs", "cp", str(stage_root / rel_path), ":" + rel_path],
            port=port,
            capture=True,
        )
        output = (result.stdout or "") + (result.stderr or "")
        if "Up to date:" not in output:
            updated_files.append(rel_path)

    if updated_files:
        status("updated files")
        for rel_path in updated_files:
            print(rel_path, flush=True)
    else:
        status("updated files: none")


def attempt_recovery(stage_root: Path, port: str) -> bool:
    status("recovery: trying soft reset")
    try:
        run_mpremote(["soft-reset"], port=port)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        status("recovery: soft reset failed")
    else:
        try:
            deploy_once(stage_root, port)
            return True
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            status("recovery: deploy still failed after soft reset")

    status("recovery: trying hard reset")
    try:
        hard_reset(port)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        status("recovery: hard reset failed")
    else:
        try:
            deploy_once(stage_root, port)
            return True
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            status("recovery: deploy still failed after hard reset")

    status("recovery: trying full remote clear")
    clear_remote_root(port)
    deploy_once(stage_root, port)
    return True


def main() -> int:
    args = parse_args()
    root = project_root()

    try:
        if shutil.which("mpremote") is None:
            fail("mpremote is not installed or not on PATH. Install it with pipx install mpremote.")

        ensure_port_ready(args.port)
        patterns = load_patterns(Path(args.config), root)
        stage_root = stage_payload(root, patterns)
        try:
            try:
                deploy_once(stage_root, args.port)
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
                print(
                    "[pico-deploy] failed: initial deploy failed: "
                    f"{type(exc).__name__}",
                    file=sys.stderr,
                )
                should_recover = prompt_yes_no(
                    "Deploy failed. Try recovery steps (soft reset, hard reset, clear Pico data, then retry)?"
                )
                if not should_recover:
                    raise
                attempt_recovery(stage_root, args.port)
        finally:
            shutil.rmtree(stage_root.parent, ignore_errors=True)

        return attach_repl(args.port)
    except subprocess.CalledProcessError as exc:
        if exc.stdout:
            sys.stdout.write(exc.stdout)
        if exc.stderr:
            sys.stderr.write(exc.stderr)
        print(f"[pico-deploy] failed: command exit {exc.returncode}", file=sys.stderr)
        return exc.returncode
    except subprocess.TimeoutExpired as exc:
        print(
            "[pico-deploy] failed: mpremote timeout while waiting for the Pico",
            file=sys.stderr,
        )
        if exc.stdout:
            sys.stdout.write(exc.stdout)
        if exc.stderr:
            sys.stderr.write(exc.stderr)
        return 124
    except DeployError as exc:
        print(f"[pico-deploy] failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
