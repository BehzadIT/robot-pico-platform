# ======================================================
# Pico / Pico 2W UART Logging Infrastructure
#
# Design goals:
# - USB REPL / IDE output must remain UNCHANGED
# - All print() output should also go to UART
# - UART output should include timestamps (ticks)
# - Structured logs go to UART only
#
# IMPORTANT (MicroPython-specific):
# - MicroPython has NO sys.stdout redirection
# - To capture all prints, overriding print() is required
# - This override is SAFE because we do not alter USB output
# ======================================================

import builtins
import time
import json
import _thread
from machine import UART, Pin

# ======================================================
# UART configuration
# ======================================================
UART_ID = 0
UART_BAUD = 115200
UART_TX_PIN = 0
UART_RX_PIN = 1

uart = UART(
    UART_ID,
    UART_BAUD,
    tx=Pin(UART_TX_PIN),
    rx=Pin(UART_RX_PIN),
)

# ======================================================
# Global log lock (FIX)
# ======================================================
_log_lock = _thread.allocate_lock()

# ======================================================
# Tick counter (ESP-IDF style)
# Used ONLY for UART output
# ======================================================
_BOOT_TICKS = time.ticks_ms()


def _ticks() -> int:
    """Milliseconds since boot (wrap-safe)."""
    return time.ticks_diff(time.ticks_ms(), _BOOT_TICKS)


# ======================================================
# Low-level UART write
# Must never raise or block execution
# ======================================================
def _uart_write_bytes(data: bytes) -> None:
    try:
        with _log_lock:
            uart.write(data)
    except Exception:
        # UART failures must never break firmware
        pass


# ======================================================
# Preserve original print()
# ======================================================
_orig_print = builtins.print


# ======================================================
# print() wrapper
#
# Behavior:
# - USB REPL output: EXACTLY the same as original print()
# - UART output: same text, but prefixed with ticks
#
# Why we reconstruct the string:
# - MicroPython does not expose the internal print buffer
# - We must rebuild what print() would emit
# ======================================================
def _mirrored_print(*args, **kwargs) -> None:
    sep = kwargs.get("sep", " ")
    end = kwargs.get("end", "\n")

    # 1) Original behavior (USB REPL, IDE tools)
    with _log_lock:
        _orig_print(*args, **kwargs)

    # 2) UART mirror with ticks
    # NOTE: ticks are added ONLY on UART, never on USB
    message = sep.join(str(a) for a in args) + end
    uart_line = f"({_ticks()}) {message}"

    _uart_write_bytes(uart_line.encode("utf-8"))


# ======================================================
# Enable / disable print mirroring
# ======================================================
def enable_uart_print_mirror() -> None:
    """
    Enable mirroring of all print() output to UART.
    Safe for MicroPython Tools and Thonny.
    """
    builtins.print = _mirrored_print


def disable_stdout_uart_mirror() -> None:
    """Restore original print()."""
    builtins.print = _orig_print


# ======================================================
# Structured UART logger (ESP-IDF style)
# These logs NEVER go to USB REPL
# ======================================================
LOG_LEVELS = {
    "E": 1,  # Error
    "W": 2,  # Warning
    "I": 3,  # Info
    "D": 4,  # Debug
    "V": 5,  # Verbose
}

_GLOBAL_LOG_LEVEL = LOG_LEVELS["D"]
_TAG_LOG_LEVELS = {}

# ANSI colors (safe over UART -> ESP32 -> terminal)
_COLORS = {
    "E": "\x1b[31m",  # Red
    "W": "\x1b[33m",  # Yellow
    "I": "\x1b[32m",  # Green
    "D": "\x1b[36m",  # Cyan
    "V": "\x1b[90m",  # Grey
}
_RESET = "\x1b[0m"


def _log(level: str, tag: str, msg: str) -> None:
    lvl = LOG_LEVELS.get(level, 0)
    allowed = _TAG_LOG_LEVELS.get(tag, _GLOBAL_LOG_LEVEL)

    if lvl > allowed:
        return

    # Ticks belong here as well (UART-only)
    if tag:
        line = f"{level} ({_ticks()}) {tag}: {msg}\n"
    else:
        line = f"{level} ({_ticks()}) {msg}\n"

    # UART (with color)
    color = _COLORS.get(level, "")
    reset = _RESET

    # 1) UART output
    _uart_write_bytes(f"{color}{line}{reset}".encode("utf-8"))

    # 2) USB REPL output (NO implicit newline)
    with _log_lock:
        _orig_print(f"{color}{line}{reset}", end="")


# ======================================================
# Public logging helpers
# ======================================================
def loge(msg: str, tag: str = "") -> None:  _log("E", tag, msg)
def logw(msg: str, tag: str = "") -> None:  _log("W", tag, msg)
def logi(msg: str, tag: str = "") -> None:  _log("I", tag, msg)
def logd(msg: str, tag: str = "") -> None:  _log("D", tag, msg)
def logv(msg: str, tag: str = "") -> None:  _log("V", tag, msg)


# ======================================================
# Telemetry channel (UART only, JSON, no ANSI)
# ======================================================
def telemetry(tag: str, **data) -> None:
    packet = {
        "t": _ticks(),
        "tag": tag,
        "data": data,
    }
    _uart_write_bytes((json.dumps(packet) + "\n").encode("utf-8"))
