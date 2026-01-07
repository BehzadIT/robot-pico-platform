import network
import time
import sys
from log import *

# Add parent directory to path to import config
sys.path.append('..')
from config import WIFI_CONFIG

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_CONFIG['ssid'], WIFI_CONFIG['password'])

    logd("Connecting to Wi-Fi...")
    timeout = 10  # seconds
    start = time.time()
    while not wlan.isconnected():
        if time.time() - start > timeout:
            raise RuntimeError("Wi-Fi connection failed.")
        time.sleep(1)

    ip = wlan.ifconfig()[0]
    logd(f"Connected. IP address: {ip}")
    return ip