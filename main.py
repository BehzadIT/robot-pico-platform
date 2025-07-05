from lib.microdot import Microdot
from server.routes import register_routes
from wifi.connection import connect_wifi

app = Microdot()
register_routes(app)
ip = connect_wifi()
print("Starting server...")
app.run(debug=True, port=80)