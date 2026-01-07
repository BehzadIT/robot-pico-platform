import network
import socket
import time

# Import config and logging
from config import WIFI_CONFIG
from log import *

# Connect to your router (station mode)
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_CONFIG['ssid'], WIFI_CONFIG['password'])

# Wait for connection
logi("Connecting to Wi-Fi...")
while not wlan.isconnected():
    time.sleep(1)

ip = wlan.ifconfig()[0]
logi(f"Connected. IP address: {ip}")

# Create HTTP server
addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)
logi(f'Listening on {ip}')

# Server loop
while True:
    try:
        cl, addr = s.accept()
        logd(f'Client connected from {addr}')
        cl_file = cl.makefile('rwb', 0)
        request_line = cl_file.readline()
        logd(f"Request: {request_line}")

        # Simple HTML response
        html = """\
HTTP/1.0 200 OK\r
Content-Type: text/html\r
\r
<!DOCTYPE html>
<html>
<head><title>Pico Robot</title></head>
<body>
<h1>Robot is Online</h1>
<p>Status: Connected to Router</p>
</body>
</html>
"""
        cl.send(html)
        cl.close()

    except Exception as e:
        loge(f'Error: {e}')