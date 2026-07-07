#!/usr/bin/env python3
"""Build, flash, reboot, and monitor Pico SDK C++ UF2 targets."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


DEFAULT_BUILD_DIR = Path("cpp/build-pico2-cross")
DEFAULT_TARGET = "robot_pico_control_no_wifi"
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
MSD_LABELS = ("RP2350", "RPI-RP2")


class PicoCppDeployError(RuntimeError):
    """Raised when the C++ deploy helper cannot continue."""


def project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def status(message: str) -> None:
    print(f"[pico-cpp-deploy] {message}", flush=True)


def run(command: list[str], *, cwd: Path, env: dict[str, str] | None = None) -> None:
    status("running: " + " ".join(command))
    subprocess.run(command, cwd=cwd, env=env, check=True)


def require_tool(name: str, guidance: str) -> str:
    path = shutil.which(name)
    if path is None:
        raise PicoCppDeployError(guidance)
    return path


def load_pico_sdk_env(root: Path, env: dict[str, str]) -> dict[str, str]:
    env = dict(env)
    if env.get("PICO_SDK_PATH"):
        return env

    env_file = root / "tools" / "pico_sdk_env.sh"
    if not env_file.exists():
        return env

    for line in env_file.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line.startswith("export PICO_SDK_PATH="):
            value = line.split("=", 1)[1].strip().strip('"').strip("'")
            if value:
                env["PICO_SDK_PATH"] = value
            break
    return env


def configure_if_needed(root: Path, build_dir: Path, env: dict[str, str]) -> None:
    cache_file = root / build_dir / "CMakeCache.txt"
    if cache_file.exists():
        return

    require_tool(
        "cmake",
        "cmake is not installed or not on PATH; install CMake before building Pico C++ firmware.",
    )

    if not env.get("PICO_SDK_PATH"):
        raise PicoCppDeployError(
            "PICO_SDK_PATH is not set. Source tools/pico_sdk_env.sh or export "
            "PICO_SDK_PATH before configuring the Pico SDK firmware build."
        )

    run(
        [
            "cmake",
            "-S",
            "cpp",
            "-B",
            str(build_dir),
            "-DROBOT_PICO_ENABLE_FIRMWARE=ON",
            "-DPICO_BOARD=pico2_w",
        ],
        cwd=root,
        env=env,
    )


def build_target(root: Path, build_dir: Path, target: str) -> None:
    env = load_pico_sdk_env(root, os.environ)
    configure_if_needed(root, build_dir, env)
    run(["cmake", "--build", str(build_dir), "--target", target], cwd=root, env=env)


def target_uf2(root: Path, build_dir: Path, target: str) -> Path:
    expected = root / build_dir / "firmware" / f"{target}.uf2"
    if expected.exists():
        return expected

    matches = sorted((root / build_dir).rglob(f"{target}.uf2"))
    if len(matches) == 1:
        return matches[0]
    if matches:
        choices = ", ".join(str(path) for path in matches)
        raise PicoCppDeployError(f"Multiple UF2 files matched target {target}: {choices}")

    if not expected.exists():
        raise PicoCppDeployError(
            f"UF2 not found for target {target} under {root / build_dir}. "
            "Run build first or pass the target/build directory that produced it."
        )
    return expected


def candidate_picotools(root: Path, build_dir: Path) -> list[Path]:
    candidates: list[Path] = []
    path_tool = shutil.which("picotool")
    if path_tool is not None:
        candidates.append(Path(path_tool))

    build_root = root / build_dir
    candidates.extend(
        [
            build_root / "_deps" / "picotool" / "picotool",
            build_root / "_deps" / "picotool-build" / "picotool",
        ]
    )

    seen: set[Path] = set()
    existing: list[Path] = []
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved not in seen and resolved.exists():
            seen.add(resolved)
            existing.append(resolved)
    return existing


def picotool_has_usb_support(picotool: Path) -> bool:
    result = subprocess.run(
        [str(picotool), "version"],
        check=False,
        capture_output=True,
        text=True,
    )
    output = (result.stdout or "") + (result.stderr or "")
    return "without USB support" not in output


def require_picotool(root: Path, build_dir: Path) -> str:
    tools = candidate_picotools(root, build_dir)
    if not tools:
        raise PicoCppDeployError(
            "picotool is not installed or available in the CMake build tree. "
            "Install a USB-enabled Raspberry Pi picotool, then retry."
        )

    for tool in tools:
        if picotool_has_usb_support(tool):
            return str(tool)

    choices = ", ".join(str(tool) for tool in tools)
    raise PicoCppDeployError(
        "found picotool, but none has USB support enabled. Checked: "
        f"{choices}. Install a USB-enabled picotool for flash/reboot commands."
    )


def flash_target(root: Path, build_dir: Path, target: str, build_first: bool) -> None:
    if build_first:
        build_target(root, build_dir, target)

    picotool = require_picotool(root, build_dir)
    uf2 = target_uf2(root, build_dir, target)
    run([picotool, "load", "-f", "-u", "-v", str(uf2)], cwd=root)


def candidate_msd_mounts() -> list[Path]:
    candidates: list[Path] = []
    user = os.environ.get("USER")
    for base in (Path("/run/media"), Path("/media")):
        if user:
            for label in MSD_LABELS:
                candidates.append(base / user / label)
        for label in MSD_LABELS:
            candidates.append(base / label)
    candidates.extend(Path("/mnt") / label for label in MSD_LABELS)
    return [path for path in candidates if path.is_dir()]


def flash_msd(root: Path, build_dir: Path, target: str, build_first: bool, mount: Path | None) -> None:
    if build_first:
        build_target(root, build_dir, target)

    uf2 = target_uf2(root, build_dir, target)
    mount_path = mount
    if mount_path is None:
        mounts = candidate_msd_mounts()
        if len(mounts) == 1:
            mount_path = mounts[0]
        elif mounts:
            choices = ", ".join(str(path) for path in mounts)
            raise PicoCppDeployError(f"multiple Pico UF2 mounts found; pass --mount. Found: {choices}")
        else:
            labels = ", ".join(MSD_LABELS)
            raise PicoCppDeployError(
                f"no Pico UF2 mass-storage mount found. Put the board in BOOTSEL and look for {labels}."
            )

    if not mount_path.is_dir():
        raise PicoCppDeployError(f"mount path does not exist or is not a directory: {mount_path}")

    destination = mount_path / uf2.name
    status(f"copying {uf2} to {destination}")
    try:
        shutil.copy2(uf2, destination)
    except OSError as exc:
        raise PicoCppDeployError(
            f"failed to copy UF2 to {mount_path}: {exc}. If the mount is read-only, "
            "remount or reconnect the Pico in BOOTSEL mode, then retry."
        ) from exc
    status("syncing copied UF2 to device")
    os.sync()
    status("UF2 copied; the Pico should reboot and re-enumerate as USB serial")


def reboot(root: Path, build_dir: Path, mode: str) -> None:
    picotool = require_picotool(root, build_dir)
    if mode == "bootsel":
        command = [picotool, "reboot", "-f", "-u"]
    else:
        command = [picotool, "reboot", "-f"]
    run(command, cwd=root)


def monitor(port: str, baud: int, timeout_s: float | None) -> int:
    try:
        import serial
    except ImportError as exc:
        raise PicoCppDeployError(
            "pyserial is not installed. Install pyserial or use an external serial terminal."
        ) from exc

    deadline = time.monotonic() + timeout_s if timeout_s is not None else None
    status(f"monitoring {port} at {baud} baud; press Ctrl-C to exit")
    with serial.Serial(port, baudrate=baud, timeout=0.1) as serial_port:
        try:
            while deadline is None or time.monotonic() < deadline:
                data = serial_port.read(serial_port.in_waiting or 1)
                if data:
                    sys.stdout.buffer.write(data)
                    sys.stdout.buffer.flush()
        except KeyboardInterrupt:
            status("monitor stopped")
            return 130
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="C++ UF2 build/flash/reboot helper for the robot Pico."
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=DEFAULT_BUILD_DIR,
        help=f"CMake firmware build directory, relative to repo root. Defaults to {DEFAULT_BUILD_DIR}.",
    )
    parser.add_argument(
        "--target",
        default=DEFAULT_TARGET,
        help=f"CMake firmware target to build or flash. Defaults to {DEFAULT_TARGET}.",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("build", help="Build the selected C++ firmware target.")

    flash_parser = subparsers.add_parser("flash", help="Flash the selected target UF2 with picotool.")
    flash_parser.add_argument(
        "--build",
        action="store_true",
        help="Build the selected target before flashing.",
    )

    flash_msd_parser = subparsers.add_parser(
        "flash-msd",
        help="Flash by copying the selected UF2 to a mounted BOOTSEL mass-storage volume.",
    )
    flash_msd_parser.add_argument(
        "--build",
        action="store_true",
        help="Build the selected target before copying the UF2.",
    )
    flash_msd_parser.add_argument(
        "--mount",
        type=Path,
        default=None,
        help="Mounted BOOTSEL volume. Auto-detects RP2350 or RPI-RP2 when omitted.",
    )

    reboot_parser = subparsers.add_parser("reboot", help="Reboot the Pico through picotool.")
    reboot_parser.add_argument(
        "--mode",
        choices=("app", "bootsel"),
        required=True,
        help="Use app for normal flash boot, or bootsel for USB BOOTSEL mode.",
    )

    monitor_parser = subparsers.add_parser("monitor", help="Attach to Pico USB serial output.")
    monitor_parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port. Defaults to {DEFAULT_PORT}.")
    monitor_parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate. Defaults to {DEFAULT_BAUD}.")
    monitor_parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="Optional monitor duration in seconds.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = project_root()

    try:
        if args.command == "build":
            build_target(root, args.build_dir, args.target)
        elif args.command == "flash":
            flash_target(root, args.build_dir, args.target, args.build)
        elif args.command == "flash-msd":
            flash_msd(root, args.build_dir, args.target, args.build, args.mount)
        elif args.command == "reboot":
            reboot(root, args.build_dir, args.mode)
        elif args.command == "monitor":
            return monitor(args.port, args.baud, args.timeout)
        else:
            raise PicoCppDeployError(f"Unsupported command: {args.command}")
    except subprocess.CalledProcessError as exc:
        print(f"[pico-cpp-deploy] failed: command exit {exc.returncode}", file=sys.stderr)
        return exc.returncode
    except PicoCppDeployError as exc:
        print(f"[pico-cpp-deploy] failed: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
