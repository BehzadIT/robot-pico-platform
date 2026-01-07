from lib.microdot import Microdot
from server.routes import register_routes
from wifi.connection import connect_wifi
from log import *

enable_uart_print_mirror()
app = Microdot()
register_routes(app)
ip = connect_wifi()
logi("Starting server...")
app.run(debug=True, port=80)
