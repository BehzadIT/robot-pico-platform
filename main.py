from lib.microdot import Microdot
from server.routes import register_routes

# --- Wi-Fi Router Connection ---
SSID = 'BIZI-HOME-24'
PASSWORD = 'AUxHHs7#2V3DZwuVwkK5'

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)

    print("Connecting to Wi-Fi...")
    timeout = 10  # seconds
    start = time.time()
    while not wlan.isconnected():
        if time.time() - start > timeout:
            raise RuntimeError("Wi-Fi connection failed.")
        time.sleep(1)

    ip = wlan.ifconfig()[0]
    print("Connected. IP address:", ip)
    return ip


app = Microdot()
register_routes(app)
# --- Start everything ---
ip = connect_wifi()
print("Starting server...")
app.run(debug=True, port=80)