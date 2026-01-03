# ---- Pico 2W: safe dual-output print (USB REPL + UART) ----
import builtins
from machine import UART, Pin

# Configure UART0 on GP0 (TX) / GP1 (RX)
UART_BAUD = 115200
uart = UART(0, UART_BAUD, tx=Pin(0), rx=Pin(1))

# Keep original print so we can always fall back
_orig_print = print

# Safety: auto-disable after repeated UART failures
_FAIL_LIMIT = 10
_fail_count = 0
_mirroring_enabled = False


def _safe_uart_write(s: str):
    """Write to UART; swallow all errors and auto-disable if it keeps failing."""
    global _fail_count, _mirroring_enabled
    try:
        if isinstance(s, bytes):
            # Reject raw binary explicitly
            s = s.decode("utf-8", errors="replace").encode("utf-8")
        else:
            s = str(s).encode("utf-8")

        uart.write(s)
        _fail_count = 0
    except Exception as e:
        _fail_count += 1
        if _fail_count == 1:
            # First failure: warn once to USB REPL only
            _orig_print(f"[uart-mirror] write failed ({e}); continuing safely…")
        if _fail_count >= _FAIL_LIMIT:
            # Too many consecutive failures → stop mirroring automatically
            _mirroring_enabled = False
            builtins.print = _orig_print
            _orig_print("[uart-mirror] disabled after repeated UART errors.")


def _dual_print(*args, **kwargs):
    """Drop-in replacement for print(): writes to USB REPL and UART."""
    # Respect normal print semantics
    sep = kwargs.get("sep", " ")
    end = kwargs.get("end", "\n")
    s = sep.join(str(a) for a in args) + end

    # 1) Always print to USB REPL (never wrapped in try)
    _orig_print(*args, **kwargs)

    # 2) Also attempt UART; errors are fully contained
    _safe_uart_write(s)


def enable_uart_log():
    """Start mirroring print() to UART safely."""
    global _mirroring_enabled
    if not _mirroring_enabled:
        builtins.print = _dual_print
        _mirroring_enabled = True
        _orig_print(f"[uart-mirror] enabled on UART0 @ {UART_BAUD} baud (TX=GP0, RX=GP1).")


def disable_uart_log():
    """Stop mirroring and restore normal print()."""
    global _mirroring_enabled
    if _mirroring_enabled:
        builtins.print = _orig_print
        _mirroring_enabled = False
        _orig_print("[uart-mirror] disabled.")
