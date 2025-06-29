from lib.microdot import Microdot
from server.routes import register_routes
from wifi.connection import connect_wifi
from services.robot_cross_coupled_pid import main_control_loop
import _thread

_thread.start_new_thread(main_control_loop, ())
app = Microdot()
register_routes(app)
ip = connect_wifi()
print("Starting server...")
app.run(debug=True, port=80)