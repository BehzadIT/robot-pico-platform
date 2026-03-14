import micropython
from lib.microdot import Microdot
from src.platform.wifi_connection import connect_wifi
from src.support.logging import enable_uart_print_mirror, logi
from src.transport.routes import register_routes


def run():
    enable_uart_print_mirror()
    # Improves traceback availability for emergency-context failures; hard wedges
    # still require watchdog recovery and lifecycle breadcrumbs for diagnosis.
    micropython.alloc_emergency_exception_buf(100)
    app = Microdot()
    register_routes(app)
    connect_wifi()
    logi("Starting server...")
    app.run(debug=False, port=80)
