from lib.microdot import Microdot
from server.routes import register_routes
from wifi.connection import connect_wifi
from logger_uart import enable_uart_print_mirror

enable_uart_print_mirror()
app = Microdot()
register_routes(app)
ip = connect_wifi()
print("Starting server...")
app.run(debug=True, port=80)
