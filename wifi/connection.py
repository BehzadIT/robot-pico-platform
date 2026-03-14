import network
import sys
import time
from log import *

# Add parent directory to path to import config
sys.path.append("..")
from config import WIFI_CONFIG

WIFI_TAG = "wifi"

DEFAULT_CONNECT_TIMEOUT_S = 20
DEFAULT_MAX_ATTEMPTS = 3
DEFAULT_PRE_RESET_DELAY_MS = 300
DEFAULT_POST_ACTIVATE_DELAY_MS = 300
DEFAULT_STATUS_POLL_MS = 250
DEFAULT_RETRY_BACKOFF_MS = 1000


def _cfg(name, default):
    return WIFI_CONFIG.get(name, default)


def _sleep_ms(duration_ms):
    if duration_ms > 0:
        time.sleep_ms(duration_ms)


def _status_name(status):
    status_pairs = (
        ("IDLE", getattr(network, "STAT_IDLE", None)),
        ("CONNECTING", getattr(network, "STAT_CONNECTING", None)),
        ("WRONG_PASSWORD", getattr(network, "STAT_WRONG_PASSWORD", None)),
        ("NO_AP_FOUND", getattr(network, "STAT_NO_AP_FOUND", None)),
        ("CONNECT_FAIL", getattr(network, "STAT_CONNECT_FAIL", None)),
        ("GOT_IP", getattr(network, "STAT_GOT_IP", None)),
    )
    for name, value in status_pairs:
        if value is not None and status == value:
            return name
    return "UNKNOWN"


def _is_terminal_failure_status(status):
    return status in (
        getattr(network, "STAT_WRONG_PASSWORD", object()),
        getattr(network, "STAT_NO_AP_FOUND", object()),
        getattr(network, "STAT_CONNECT_FAIL", object()),
    )


def _log_status_transition(attempt, status, connected):
    status_name = _status_name(status)
    logd(
        "Attempt %s status changed: %s (%s), connected=%s"
        % (attempt, status_name, status, connected),
        WIFI_TAG,
    )
    telemetry(
        "wifi_status",
        attempt=attempt,
        status=status,
        status_name=status_name,
        connected=connected,
    )


def _reset_interface(wlan):
    try:
        wlan.disconnect()
    except Exception as exc:
        logd("disconnect() ignored during reset: %s" % exc, WIFI_TAG)

    try:
        wlan.active(False)
    except Exception as exc:
        logd("active(False) ignored during reset: %s" % exc, WIFI_TAG)

    _sleep_ms(_cfg("pre_reset_delay_ms", DEFAULT_PRE_RESET_DELAY_MS))
    wlan.active(True)
    _sleep_ms(_cfg("post_activate_delay_ms", DEFAULT_POST_ACTIVATE_DELAY_MS))


def _ifconfig_dict(wlan):
    try:
        ip, subnet, gateway, dns = wlan.ifconfig()
        return {
            "ip": ip,
            "subnet": subnet,
            "gateway": gateway,
            "dns": dns,
        }
    except Exception as exc:
        logd("ifconfig() unavailable: %s" % exc, WIFI_TAG)
        return None


def _wait_for_connection(wlan, attempt, timeout_s):
    start_ms = time.ticks_ms()
    timeout_ms = int(timeout_s * 1000)
    poll_ms = _cfg("status_poll_ms", DEFAULT_STATUS_POLL_MS)
    last_status = None

    while True:
        connected = wlan.isconnected()
        try:
            status = wlan.status()
        except Exception:
            status = None

        if status != last_status:
            _log_status_transition(attempt, status, connected)
            last_status = status

        if connected:
            return True, status

        if _is_terminal_failure_status(status):
            return False, status

        elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
        if elapsed_ms >= timeout_ms:
            return False, status

        _sleep_ms(poll_ms)


def connect_wifi():
    ssid = WIFI_CONFIG["ssid"]
    password = WIFI_CONFIG["password"]
    connect_timeout_s = _cfg("connect_timeout_s", DEFAULT_CONNECT_TIMEOUT_S)
    max_attempts = _cfg("max_attempts", DEFAULT_MAX_ATTEMPTS)
    retry_backoff_ms = _cfg("retry_backoff_ms", DEFAULT_RETRY_BACKOFF_MS)
    wlan = network.WLAN(network.STA_IF)
    last_status = None

    logi(
        "Connecting to Wi-Fi SSID '%s' (timeout=%ss, attempts=%s)"
        % (ssid, connect_timeout_s, max_attempts),
        WIFI_TAG,
    )

    for attempt in range(1, max_attempts + 1):
        telemetry("wifi_attempt", attempt=attempt, max_attempts=max_attempts, ssid=ssid)
        logi("Attempt %s/%s starting" % (attempt, max_attempts), WIFI_TAG)

        try:
            pre_status = wlan.status()
        except Exception:
            pre_status = None

        logd(
            "Pre-reset state: active=%s connected=%s status=%s (%s)"
            % (wlan.active(), wlan.isconnected(), _status_name(pre_status), pre_status),
            WIFI_TAG,
        )

        _reset_interface(wlan)
        wlan.connect(ssid, password)

        connected, last_status = _wait_for_connection(wlan, attempt, connect_timeout_s)
        if connected:
            cfg = _ifconfig_dict(wlan)
            ip = cfg["ip"] if cfg else wlan.ifconfig()[0]
            logi("Connected on attempt %s. IP address: %s" % (attempt, ip), WIFI_TAG)
            if cfg:
                logd(
                    "IP configuration: ip=%s subnet=%s gateway=%s dns=%s"
                    % (cfg["ip"], cfg["subnet"], cfg["gateway"], cfg["dns"]),
                    WIFI_TAG,
                )
            telemetry("wifi_connected", attempt=attempt, ip=ip, ifconfig=cfg)
            return ip

        last_status_name = _status_name(last_status)
        elapsed_reason = "terminal_status" if _is_terminal_failure_status(last_status) else "timeout"

        logw(
            "Attempt %s/%s failed: %s (%s), reason=%s"
            % (attempt, max_attempts, last_status_name, last_status, elapsed_reason),
            WIFI_TAG,
        )
        telemetry(
            "wifi_failed",
            attempt=attempt,
            status=last_status,
            status_name=last_status_name,
            reason=elapsed_reason,
        )

        if attempt < max_attempts:
            logi("Retrying in %sms" % retry_backoff_ms, WIFI_TAG)
            _sleep_ms(retry_backoff_ms)

    last_status_name = _status_name(last_status)
    telemetry(
        "wifi_giveup",
        attempts=max_attempts,
        last_status=last_status,
        last_status_name=last_status_name,
    )
    loge(
        "Wi-Fi connection failed after %s attempts. Last status: %s (%s)"
        % (max_attempts, last_status_name, last_status),
        WIFI_TAG,
    )
    raise RuntimeError(
        "Wi-Fi connection failed after %s attempts; last status: %s (%s)"
        % (max_attempts, last_status_name, last_status)
    )
