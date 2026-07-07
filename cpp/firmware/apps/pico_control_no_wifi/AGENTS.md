# AGENTS.md

## Purpose
This directory contains the fresh Pico SDK clean app for the no-network control
firmware path.

## Read First
- `AGENTIC_DEVELOPMENT.md`
- `TOOLS.md`
- `../../README.md`
- `../../../README.md`

## Working Rules
- Keep this app independent from the existing migrated C++ runtime unless the
  task explicitly says to integrate them.
- Preserve the Pico ownership boundary: Pico owns timing-sensitive control and
  safety; this clean app currently owns only boot, USB stdio, and bench-loop
  proof of life.
- After every change, build the app yourself before handing work back.
- If the change has automatic verification, run it and report the result.
- If the change needs human bench observation, provide the exact command for the
  user to run and the expected output to look for.
- Prefer `python3 tools/pico_cpp_deploy.py monitor --timeout ...` for
  agent-run evidence capture because it records logs in command output and
  reconnects across Pico reboot. Use `minicom` for human interactive or
  long-running observation.
