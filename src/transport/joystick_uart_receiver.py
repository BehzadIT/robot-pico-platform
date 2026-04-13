import _thread

import utime
from machine import Pin, UART

from settings.config import CONTROLLER_DRIVE_CONFIG, CONTROLLER_UART_CONFIG
from src.control.controller_input_mapping import ControllerDriveMapper
from src.protocol.request_models import ApiDriveRequest
from src.protocol.joystick_transport import JoystickFrameParser
from src.support.logging import loge, logi, logw, telemetry

_lock = _thread.allocate_lock()
_latest_snapshot = None
_receiver_started = False
_status = {
    "started": False,
    "frames_ok": 0,
    "bytes_rx": 0,
    "last_seq": None,
    "last_source_uptime_ms": None,
    "last_receive_ms": None,
    "controller_mode": "inactive",
    "controller_driving": False,
    "controller_last_stop_reason": None,
    "controller_last_snapshot_seq": None,
    "controller_last_throttle_percent": 0,
    "controller_last_turn_percent": 0,
    "controller_last_age_ms": None,
}

_BUTTON_NAMES = (
    (0x0001, "A"),
    (0x0002, "B"),
    (0x0004, "X"),
    (0x0008, "Y"),
    (0x0010, "Left Bumper"),
    (0x0020, "Right Bumper"),
    (0x0040, "Left Trigger Button"),
    (0x0080, "Right Trigger Button"),
    (0x0100, "Left Stick Press"),
    (0x0200, "Right Stick Press"),
)

_MISC_BUTTON_NAMES = (
    (0x01, "Xbox"),
    (0x02, "View"),
    (0x04, "Menu"),
    (0x08, "Capture"),
)

_DPAD_NAMES = (
    (0x01, "D-Pad Up"),
    (0x02, "D-Pad Down"),
    (0x04, "D-Pad Right"),
    (0x08, "D-Pad Left"),
)

_STICK_DELTA_THRESHOLD = 20
_TRIGGER_DELTA_THRESHOLD = 8
CONTROLLER_DRIVE_TAG = "controller_drive"


def _copy_status():
    return {
        "started": _status["started"],
        "frames_ok": _status["frames_ok"],
        "bytes_rx": _status["bytes_rx"],
        "last_seq": _status["last_seq"],
        "last_source_uptime_ms": _status["last_source_uptime_ms"],
        "last_receive_ms": _status["last_receive_ms"],
        "controller_mode": _status["controller_mode"],
        "controller_driving": _status["controller_driving"],
        "controller_last_stop_reason": _status["controller_last_stop_reason"],
        "controller_last_snapshot_seq": _status["controller_last_snapshot_seq"],
        "controller_last_throttle_percent": _status["controller_last_throttle_percent"],
        "controller_last_turn_percent": _status["controller_last_turn_percent"],
        "controller_last_age_ms": _status["controller_last_age_ms"],
    }


def _names_from_bits(value, mapping):
    names = []
    for bit, name in mapping:
        if value & bit:
            names.append(name)
    return names


def _pressed_names(previous, current, mapping):
    changed = (~previous) & current
    return _names_from_bits(changed, mapping)


def _released_names(previous, current, mapping):
    changed = previous & (~current)
    return _names_from_bits(changed, mapping)


def latest_joystick_snapshot():
    with _lock:
        return _latest_snapshot


def joystick_uart_status_snapshot():
    with _lock:
        return _copy_status()


def controller_drive_status_snapshot():
    with _lock:
        return {
            "started": _status["started"],
            "mode": _status["controller_mode"],
            "driving": _status["controller_driving"],
            "last_stop_reason": _status["controller_last_stop_reason"],
            "last_snapshot_seq": _status["controller_last_snapshot_seq"],
            "last_throttle_percent": _status["controller_last_throttle_percent"],
            "last_turn_percent": _status["controller_last_turn_percent"],
            "last_age_ms": _status["controller_last_age_ms"],
        }


def _apply_snapshot(snapshot):
    global _latest_snapshot
    previous = None
    with _lock:
        previous = _latest_snapshot
        _latest_snapshot = snapshot
        _status["frames_ok"] += 1
        _status["last_seq"] = snapshot.seq
        _status["last_source_uptime_ms"] = snapshot.source_uptime_ms
        _status["last_receive_ms"] = snapshot.received_at_ms
    return previous


def _next_drive_seq():
    from src.control.drive_command_manager import drive_command_manager

    with _lock:
        candidate = max(int(utime.ticks_ms()), int(_status["last_seq"] or 0)) + 1
    while not drive_command_manager.is_fresh_sequence(candidate):
        candidate += 1
    return candidate


def _record_controller_mode(mode, reason=None, mapped=None, is_driving=None):
    with _lock:
        _status["controller_mode"] = mode
        _status["controller_last_stop_reason"] = reason
        if is_driving is not None:
            _status["controller_driving"] = is_driving
        if mapped is not None:
            _status["controller_last_snapshot_seq"] = mapped.get("snapshot_seq")
            _status["controller_last_throttle_percent"] = mapped.get("throttle_percent", 0)
            _status["controller_last_turn_percent"] = mapped.get("turn_percent", 0)
            _status["controller_last_age_ms"] = mapped.get("age_ms")
        elif mode == "inactive":
            _status["controller_last_throttle_percent"] = 0
            _status["controller_last_turn_percent"] = 0
            _status["controller_last_age_ms"] = None


def _controller_is_driving():
    with _lock:
        return _status["controller_driving"]


def _log_controller_mode_transition(previous_mode, mapped):
    new_mode = mapped["mode"]
    if previous_mode == new_mode:
        return

    if new_mode == "driving":
        logi(
            "Controller drive active throttle=%s turn=%s seq=%s age_ms=%s"
            % (
                mapped["throttle_percent"],
                mapped["turn_percent"],
                mapped["snapshot_seq"],
                mapped["age_ms"],
            ),
            CONTROLLER_DRIVE_TAG,
        )
        telemetry(
            "controller_drive_mode",
            mode=new_mode,
            throttle_percent=mapped["throttle_percent"],
            turn_percent=mapped["turn_percent"],
            snapshot_seq=mapped["snapshot_seq"],
            age_ms=mapped["age_ms"],
        )
        return

    logi(
        "Controller mode=%s reason=%s age_ms=%s"
        % (new_mode, mapped.get("reason"), mapped.get("age_ms")),
        CONTROLLER_DRIVE_TAG,
    )
    telemetry(
        "controller_drive_mode",
        mode=new_mode,
        reason=mapped.get("reason"),
        age_ms=mapped.get("age_ms", 0),
    )


def _publish_controller_stop(reason):
    from src.control.drive_command_manager import drive_command_manager
    from src.control.drivetrain_controller import drivetrain_controller

    if not _controller_is_driving():
        return

    drive_command_manager.stop(reason=reason)
    drivetrain_controller.request_stop(reason=reason)
    with _lock:
        _status["controller_driving"] = False
    logi("Controller stop requested: %s" % reason, CONTROLLER_DRIVE_TAG)
    telemetry("controller_drive_stop", reason=reason)


def _publish_controller_drive(mapped):
    from src.control.drive_command_manager import drive_command_manager
    from src.control.drivetrain_controller import drivetrain_controller

    drive_request = ApiDriveRequest.from_intent(
        throttle_percent=mapped["throttle_percent"],
        turn_percent=mapped["turn_percent"],
        seq=_next_drive_seq(),
        sent_at_ms=utime.ticks_ms(),
    )
    drive_result = drivetrain_controller.request_drive(drive_command_manager, drive_request)
    if not drive_result["ok"]:
        reason = "drive_rejected_%s" % drive_result["code"]
        _publish_controller_stop(reason)
        logw(
            "Controller drive rejected code=%s detail=%s"
            % (drive_result["code"], drive_result.get("detail")),
            CONTROLLER_DRIVE_TAG,
        )
        return

    _record_controller_mode("driving", mapped=mapped, is_driving=True)


def _log_input_transition(previous, current):
    if previous is None:
        return
    if (
        previous.buttons == current.buttons
        and previous.misc_buttons == current.misc_buttons
        and previous.dpad == current.dpad
    ):
        return

    parts = []

    pressed = _pressed_names(previous.buttons, current.buttons, _BUTTON_NAMES)
    released = _released_names(previous.buttons, current.buttons, _BUTTON_NAMES)
    if pressed:
        parts.append("pressed=%s" % ("[" + ",".join(pressed) + "]"))
    if released:
        parts.append("released=%s" % ("[" + ",".join(released) + "]"))

    misc_pressed = _pressed_names(previous.misc_buttons, current.misc_buttons, _MISC_BUTTON_NAMES)
    misc_released = _released_names(
        previous.misc_buttons, current.misc_buttons, _MISC_BUTTON_NAMES
    )
    if misc_pressed:
        parts.append("misc_pressed=%s" % ("[" + ",".join(misc_pressed) + "]"))
    if misc_released:
        parts.append("misc_released=%s" % ("[" + ",".join(misc_released) + "]"))

    dpad_pressed = _pressed_names(previous.dpad, current.dpad, _DPAD_NAMES)
    dpad_released = _released_names(previous.dpad, current.dpad, _DPAD_NAMES)
    if dpad_pressed:
        parts.append("dpad_pressed=%s" % ("[" + ",".join(dpad_pressed) + "]"))
    if dpad_released:
        parts.append("dpad_released=%s" % ("[" + ",".join(dpad_released) + "]"))

    if not parts:
        return

    logi("Joystick UART input seq=%s %s" % (current.seq, " ".join(parts)))


def _analog_changed(previous_value, current_value, threshold):
    if previous_value == current_value:
        return False
    if abs(current_value - previous_value) >= threshold:
        return True
    previous_active = abs(previous_value) >= threshold
    current_active = abs(current_value) >= threshold
    return previous_active != current_active


def _trigger_changed(previous_value, current_value, threshold):
    if previous_value == current_value:
        return False
    if abs(current_value - previous_value) >= threshold:
        return True
    previous_active = previous_value >= threshold
    current_active = current_value >= threshold
    return previous_active != current_active


def _stick_changed(previous_x, previous_y, current_x, current_y):
    return _analog_changed(previous_x, current_x, _STICK_DELTA_THRESHOLD) or _analog_changed(
        previous_y, current_y, _STICK_DELTA_THRESHOLD
    )


def _log_snapshot_delta(previous, current):
    changed_parts = []

    if previous is None:
        if current.connected is False:
            changed_parts.append("connected=False")
        if current.left_stick_x or current.left_stick_y:
            changed_parts.append(
                "left_stick(x=%s y=%s)" % (current.left_stick_x, current.left_stick_y)
            )
        if current.right_stick_x or current.right_stick_y:
            changed_parts.append(
                "right_stick(x=%s y=%s)" % (current.right_stick_x, current.right_stick_y)
            )
        if current.left_trigger:
            changed_parts.append("left_trigger=%s" % current.left_trigger)
        if current.right_trigger:
            changed_parts.append("right_trigger=%s" % current.right_trigger)
    else:
        if previous.connected != current.connected:
            changed_parts.append("connected=%s" % current.connected)
        if _stick_changed(
            previous.left_stick_x,
            previous.left_stick_y,
            current.left_stick_x,
            current.left_stick_y,
        ):
            changed_parts.append(
                "left_stick(x=%s y=%s)" % (current.left_stick_x, current.left_stick_y)
            )
        if _stick_changed(
            previous.right_stick_x,
            previous.right_stick_y,
            current.right_stick_x,
            current.right_stick_y,
        ):
            changed_parts.append(
                "right_stick(x=%s y=%s)" % (current.right_stick_x, current.right_stick_y)
            )
        if _trigger_changed(previous.left_trigger, current.left_trigger, _TRIGGER_DELTA_THRESHOLD):
            changed_parts.append("left_trigger=%s" % current.left_trigger)
        if _trigger_changed(
            previous.right_trigger, current.right_trigger, _TRIGGER_DELTA_THRESHOLD
        ):
            changed_parts.append("right_trigger=%s" % current.right_trigger)

    if not changed_parts:
        return

    age_ms = utime.ticks_diff(utime.ticks_ms(), current.received_at_ms)
    logi("Joystick UART input seq=%s %s age_ms=%s" % (current.seq, " ".join(changed_parts), age_ms))


def _receiver_loop():
    """Run the single Pico controller pipeline worker.

    This worker owns:
    - UART read and frame parsing
    - latest controller snapshot/status storage
    - controller mode classification through `ControllerDriveMapper`
    - fixed-cadence drive republish while input is valid
    - stop requests when input becomes stale, disconnected, or neutral

    It replaces the earlier two-thread model because RP2040 MicroPython cannot
    reliably host both a UART receiver worker and a separate controller adapter
    worker at the same time.
    """
    cfg = CONTROLLER_UART_CONFIG
    drive_cfg = CONTROLLER_DRIVE_CONFIG
    parser = JoystickFrameParser()
    mapper = ControllerDriveMapper(drive_cfg)
    uart = UART(
        cfg["uart_id"],
        cfg["baud"],
        tx=Pin(cfg["tx_pin"]),
        rx=Pin(cfg["rx_pin"]),
    )
    with _lock:
        _status["started"] = True

    logi(
        "Joystick UART receiver started uart=%s tx=%s rx=%s baud=%s"
        % (cfg["uart_id"], cfg["tx_pin"], cfg["rx_pin"], cfg["baud"])
    )
    logi(
        "Controller pipeline active poll_ms=%s publish_period_ms=%s stale_ms=%s"
        % (cfg["poll_ms"], drive_cfg["publish_period_ms"], drive_cfg["snapshot_stale_ms"]),
        CONTROLLER_DRIVE_TAG,
    )
    last_publish_ms = utime.ticks_ms()

    while True:
        try:
            available = uart.any()
            if available:
                chunk = uart.read(min(available, cfg["read_chunk"]))
                if chunk:
                    now_ms = utime.ticks_ms()
                    with _lock:
                        _status["bytes_rx"] += len(chunk)
                    snapshots = parser.feed(chunk, received_at_ms=now_ms)
                    for snapshot in snapshots:
                        previous = _apply_snapshot(snapshot)
                        _log_input_transition(previous, snapshot)
                        _log_snapshot_delta(previous, snapshot)

            now_ms = utime.ticks_ms()
            if utime.ticks_diff(now_ms, last_publish_ms) >= int(drive_cfg["publish_period_ms"]):
                snapshot = latest_joystick_snapshot()
                previous_mode = controller_drive_status_snapshot()["mode"]
                mapped = mapper.map_snapshot(snapshot, now_ms=now_ms)
                _log_controller_mode_transition(previous_mode, mapped)

                if mapped["mode"] == "driving":
                    _publish_controller_drive(mapped)
                else:
                    _record_controller_mode(
                        mapped["mode"],
                        reason=mapped.get("reason"),
                        mapped=mapped,
                        is_driving=False if mapped["mode"] == "inactive" else _controller_is_driving(),
                    )
                    if mapped["mode"] in ("inactive", "armed_neutral"):
                        _publish_controller_stop(mapped.get("reason", mapped["mode"]))

                last_publish_ms = now_ms

            utime.sleep_ms(cfg["poll_ms"])
        except Exception as exc:
            logw("Joystick UART receiver loop error: %s" % exc)
            mapper.reset()
            utime.sleep_ms(100)


def start_joystick_uart_receiver():
    global _receiver_started

    cfg = CONTROLLER_UART_CONFIG
    if not cfg.get("enabled", False):
        logi("Joystick UART receiver disabled by config")
        return True

    if _receiver_started:
        return True

    try:
        _thread.start_new_thread(_receiver_loop, ())
        _receiver_started = True
        return True
    except Exception as exc:
        loge("Failed to start joystick UART receiver: %s" % exc)
        return False
