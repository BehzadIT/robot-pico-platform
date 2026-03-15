import micropython
import machine
from microdot import Microdot
from src.platform.wifi_connection import connect_wifi
from src.support.logging import enable_uart_print_mirror, logi
from src.transport.routes import register_routes


def _reset_cause_name():
    try:
        cause = machine.reset_cause()
    except Exception:
        return "unknown"

    cause_names = {}
    for attr in ("PWRON_RESET", "HARD_RESET", "WDT_RESET", "DEEPSLEEP_RESET", "SOFT_RESET"):
        value = getattr(machine, attr, None)
        if value is not None:
            cause_names[value] = attr.lower()
    return cause_names.get(cause, str(cause))


def run():
    enable_uart_print_mirror()
    # Improves traceback availability for emergency-context failures; hard wedges
    # still require watchdog recovery and lifecycle breadcrumbs for diagnosis.
    micropython.alloc_emergency_exception_buf(100)
    logi("Boot reset cause: %s" % _reset_cause_name())
    app = Microdot()
    register_routes(app)
    connect_wifi()
    logi("Starting server...")
    app.run(debug=False, port=80)
