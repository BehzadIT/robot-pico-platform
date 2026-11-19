#!/usr/bin/env python3
"""US-003 adapter for the Pico real-time control platform."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = [
    {"id": "pico-host-default", "description": "host CMake build and CTest", "gate": 0, "level": "host"},
    {"id": "pico-firmware-compile", "description": "compile the clean no-network Pico target", "gate": 0, "level": "compile"},
    {"id": "PICO-BOOT-001", "description": "flash the clean app and assert boot and both task heartbeats", "gate": 1, "level": "device"},
    {"id": "PICO-STALE-001", "description": "host stale-command safety assertions", "gate": 0, "level": "host"},
    {"id": "UART-FRAME-001", "description": "host valid and malformed UART frame assertions", "gate": 0, "level": "host"},
    {"id": "ENCODER-SIGN-001", "description": "safe-bench encoder sign qualification", "gate": 3, "level": "physical"},
    {"id": "STOP-001", "description": "safe-bench stop qualification", "gate": 3, "level": "physical"},
    {"id": "RECOVER-001", "description": "safe-bench explicit recovery qualification", "gate": 3, "level": "physical"},
    {"id": "ESTOP-001", "description": "safe-bench emergency disconnect qualification", "gate": 3, "level": "physical"},
]


def now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def base(command: str, level: str = "static") -> dict[str, Any]:
    stamp = now()
    return {"schema_version": 1, "command": command, "scenario": None, "scenario_version": None, "started_at": stamp, "finished_at": stamp, "outcome": "pass", "verification_level": level, "physical_stage": "safe_bench" if level == "physical" else None, "environment": {}, "repositories": [], "devices": [], "checks": [], "artifacts": [], "approvals": [], "evidence_id": None, "safe_next_action": None}


def item(check_id: str, outcome: str, expected: str, observed: str, duration: int = 0, category: str | None = None, code: int | None = None, command: list[str] | None = None) -> dict[str, Any]:
    return {"id": check_id, "outcome": outcome, "expected_signal": expected, "observed": observed[-4000:], "duration_ms": duration, "failure_category": category, "exit_code": code, "command": command}


def finish(value: dict[str, Any]) -> dict[str, Any]:
    order = {"pass": 0, "skipped": 1, "inconclusive": 2, "blocked": 3, "fail": 4}
    value["outcome"] = max((entry["outcome"] for entry in value["checks"]), key=order.get) if value["checks"] else "inconclusive"
    value["finished_at"] = now()
    return value


def run(command: list[str], timeout: int = 1800) -> tuple[subprocess.CompletedProcess[str], int]:
    started = time.monotonic()
    env = dict(os.environ)
    if not env.get("PICO_SDK_PATH"):
        candidate = Path.home() / "Projects" / "pico-sdk"
        if candidate.is_dir():
            env["PICO_SDK_PATH"] = str(candidate)
    try:
        completed = subprocess.run(command, cwd=ROOT, env=env, text=True, capture_output=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired as exc:
        completed = subprocess.CompletedProcess(command, 124, exc.stdout or "", exc.stderr or "timeout")
    return completed, int((time.monotonic() - started) * 1000)


def repo() -> dict[str, Any]:
    sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, capture_output=True, check=False).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain"], cwd=ROOT, text=True, capture_output=True, check=False).stdout.strip())
    return {"name": "pico", "sha": sha, "dirty": dirty}


def describe() -> dict[str, Any]:
    value = base("pico adapter describe")
    value["environment"] = {"adapter_protocol": 1, "platform": "pico", "scenarios": SCENARIOS}
    value["checks"].append(item("pico.adapter", "pass", "adapter protocol 1", "adapter protocol 1"))
    return finish(value)


def doctor() -> dict[str, Any]:
    value = base("pico doctor")
    value["repositories"] = [repo()]
    for name in ("cmake", "picotool", "mpremote"):
        path = shutil.which(name)
        required = name != "mpremote"
        outcome = "pass" if path else "blocked" if required else "skipped"
        value["checks"].append(item(f"pico.tool.{name}", outcome, f"{name} available", path or "not found", category=None if path else "missing_tool" if required else None))
    sdk = Path(os.environ.get("PICO_SDK_PATH", str(Path.home() / "Projects" / "pico-sdk")))
    value["checks"].append(item("pico.sdk", "pass" if sdk.is_dir() else "blocked", "Pico SDK directory", str(sdk) if sdk.is_dir() else "not found", category=None if sdk.is_dir() else "environment_missing"))
    value["environment"] = {"platform": "pico", "board": "pico2_w", "default_port": "/dev/ttyACM0", "software_tools": [{"name": "pico-sdk", "version": "2.2.0", "source": "Pico SDK checkout"}], "read_only": True}
    return finish(value)


def command_check(value: dict[str, Any], check_id: str, command: list[str], expected: str, category: str) -> bool:
    completed, duration = run(command)
    outcome = "pass" if completed.returncode == 0 else "fail"
    value["checks"].append(item(check_id, outcome, expected, (completed.stdout + "\n" + completed.stderr).strip(), duration, None if outcome == "pass" else category, completed.returncode, command))
    return outcome == "pass"


def verify(scenario: str, device_json: str | None) -> dict[str, Any]:
    level = next((entry["level"] for entry in SCENARIOS if entry["id"] == scenario), "host")
    value = base(f"pico verify {scenario}", level)
    value["scenario"] = scenario
    value["scenario_version"] = 1
    value["repositories"] = [repo()]
    value["environment"] = {"platform": "pico", "board": "pico2_w", "software_tools": [{"name": "pico-sdk", "version": "2.2.0", "source": "Pico SDK checkout"}]}
    if scenario == "pico-host-default":
        if command_check(value, "pico.host.configure", ["cmake", "-S", "cpp", "-B", "cpp/build-host"], "host CMake configure succeeds", "build_failed"):
            if command_check(value, "pico.host.build", ["cmake", "--build", "cpp/build-host"], "host build succeeds", "build_failed"):
                command_check(value, "pico.host.ctest", ["ctest", "--test-dir", "cpp/build-host", "--output-on-failure"], "all host tests pass", "assertion_failed")
    elif scenario == "pico-firmware-compile":
        command_check(value, "pico.firmware.build", [sys.executable, "tools/pico_cpp_deploy.py", "build"], "clean no-network target builds", "build_failed")
    elif scenario in {"PICO-STALE-001", "UART-FRAME-001"}:
        if command_check(value, "pico.host.configure", ["cmake", "-S", "cpp", "-B", "cpp/build-host"], "host CMake configure succeeds", "build_failed"):
            if command_check(value, "pico.host.build", ["cmake", "--build", "cpp/build-host"], "host build succeeds", "build_failed"):
                command_check(value, f"pico.scenario.{scenario.lower()}", ["ctest", "--test-dir", "cpp/build-host", "--output-on-failure"], f"host suite containing {scenario} assertions passes", "assertion_failed")
    elif scenario == "PICO-BOOT-001":
        if not device_json:
            value["checks"].append(item("pico.device", "blocked", "resolved stable Pico serial device", "device metadata was not supplied", category="device_absent"))
            return finish(value)
        device_data = json.loads(Path(device_json).read_text(encoding="utf-8"))
        port = device_data["observed"]["port"]
        monitor_command = [sys.executable, "tools/pico_cpp_deploy.py", "monitor", "--port", port, "--timeout", "35"]
        started = time.monotonic()
        monitor = subprocess.Popen(monitor_command, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        time.sleep(0.5)
        flash, flash_duration = run([sys.executable, "tools/pico_cpp_deploy.py", "flash", "--build"], 1800)
        try:
            monitor_output, _ = monitor.communicate(timeout=40)
        except subprocess.TimeoutExpired:
            monitor.terminate()
            monitor_output, _ = monitor.communicate(timeout=5)
        value["checks"].append(item("pico.boot.flash", "pass" if flash.returncode == 0 else "fail", "identified clean app flashes successfully", (flash.stdout + "\n" + flash.stderr).strip(), flash_duration, None if flash.returncode == 0 else "flash_failed", flash.returncode, [sys.executable, "tools/pico_cpp_deploy.py", "flash", "--build"]))
        expected_lines = ["robot_pico_control_no_wifi: boot", "service started core=0", "control started core=1", "service heartbeat core=0", "control heartbeat core=1"]
        missing = [line for line in expected_lines if line not in monitor_output]
        value["checks"].append(item("pico.boot.heartbeat", "pass" if not missing else "fail", "boot plus service/control start and heartbeat lines", monitor_output if not missing else f"missing {missing}\n{monitor_output}", int((time.monotonic() - started) * 1000), None if not missing else "boot_timeout", monitor.returncode, monitor_command))
    else:
        value["checks"].append(item(f"pico.physical.{scenario.lower()}", "blocked", "qualified physical procedure and assertions", "hardware questions, fixtures, and retained bench evidence are required", category="unsafe_precondition"))
        value["safe_next_action"] = "Resolve the linked US-002 hardware facts and execute the versioned safe-bench procedure."
    return finish(value)


def main() -> int:
    as_json = "--json" in sys.argv
    argv = [arg for arg in sys.argv[1:] if arg != "--json"]
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("describe")
    sub.add_parser("doctor")
    sub.add_parser("inventory")
    verify_parser = sub.add_parser("verify")
    verify_parser.add_argument("scenario")
    verify_parser.add_argument("--device-json")
    args = parser.parse_args(argv)
    value = describe() if args.command == "describe" else doctor() if args.command in {"doctor", "inventory"} else verify(args.scenario, args.device_json)
    print(json.dumps(value, indent=2, sort_keys=True) if as_json else f"{value['command']}: {value['outcome']}")
    return 0 if value["outcome"] == "pass" else 20 if value["outcome"] == "blocked" else 10


if __name__ == "__main__":
    raise SystemExit(main())
