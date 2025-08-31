from lib.microdot import Microdot
from server.routes import register_routes
from wifi.connection import connect_wifi
from logger_uart import enable_uart_log

enable_uart_log()
app = Microdot()
register_routes(app)
ip = connect_wifi()
print("Starting server...")
app.run(debug=True, port=80)
