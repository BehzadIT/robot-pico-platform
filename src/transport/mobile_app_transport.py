import uasyncio as asyncio
import ujson
import utime
from microdot.websocket import with_websocket

from src.control.drive_command_manager import drive_command_manager
from src.control.drivetrain_constants import SafetyTiming
from src.control.drivetrain_controller import drivetrain_controller
from src.protocol.request_models import ApiDriveRequest
from src.support.logging import loge, logi, logw

CMD_DRIVE = "d"
CMD_STOP = "s"
CMD_RECOVER = "r"
ACK_STOP = "s"
ACK_RECOVER = "r"
ERR_INVALID_JSON = "ij"
ERR_STALE = "st"
ERR_BAD_DRIVE = "bd"
ERR_BAD_STOP = "bs"
ERR_BAD_RECOVER = "br"
ERR_INVALID_COMMAND_SOURCE = "ic"
ERR_UNKNOWN_COMMAND = "uc"
ERR_HARDWARE_FAULT = "hf"
ERR_RECOVERY_IN_PROGRESS = "ri"
ERR_STOP_IN_PROGRESS = "si"
ERR_RECOVERY_CONFLICT = "rc"
ERR_RECOVERY_FAILED = "rf"
ERR_STOP_FAILED = "sf"
DRIVE_LOG_SAMPLE_MS = 500


def _print_exception(exc):
    try:
        import sys

        sys.print_exception(exc)
    except Exception:
        pass


async def _wait_for_stop_completion(seq, timeout_ms=SafetyTiming.STOP_COMPLETION_TIMEOUT_MS):
    deadline = utime.ticks_add(utime.ticks_ms(), timeout_ms)
    while utime.ticks_diff(deadline, utime.ticks_ms()) > 0:
        if drivetrain_controller.is_stop_completed(seq):
            return True
        if drivetrain_controller.stop_error(seq) is not None:
            return False
        await asyncio.sleep_ms(10)
    return drivetrain_controller.is_stop_completed(seq)


async def _wait_for_recover_completion(seq, timeout_ms=SafetyTiming.RECOVER_COMPLETION_TIMEOUT_MS):
    deadline = utime.ticks_add(utime.ticks_ms(), timeout_ms)
    while utime.ticks_diff(deadline, utime.ticks_ms()) > 0:
        if drivetrain_controller.is_recover_completed(seq):
            return True
        if drivetrain_controller.recover_error(seq) is not None:
            return False
        await asyncio.sleep_ms(10)
    return drivetrain_controller.is_recover_completed(seq)


def _wait_for_stop_completion_blocking(seq, timeout_ms=SafetyTiming.STOP_COMPLETION_TIMEOUT_MS):
    deadline = utime.ticks_add(utime.ticks_ms(), timeout_ms)
    while utime.ticks_diff(deadline, utime.ticks_ms()) > 0:
        if drivetrain_controller.is_stop_completed(seq):
            return True
        if drivetrain_controller.stop_error(seq) is not None:
            return False
        utime.sleep_ms(10)
    return drivetrain_controller.is_stop_completed(seq)


def _wait_for_recover_completion_blocking(seq, timeout_ms=SafetyTiming.RECOVER_COMPLETION_TIMEOUT_MS):
    deadline = utime.ticks_add(utime.ticks_ms(), timeout_ms)
    while utime.ticks_diff(deadline, utime.ticks_ms()) > 0:
        if drivetrain_controller.is_recover_completed(seq):
            return True
        if drivetrain_controller.recover_error(seq) is not None:
            return False
        utime.sleep_ms(10)
    return drivetrain_controller.is_recover_completed(seq)


def _ws_log(event, **data):
    safe_data = ", ".join(["%s=%s" % (key, data[key]) for key in data])
    logi("[ws] %s%s" % (event, (" " + safe_data) if safe_data else ""))


def _should_log_drive_sample(state, drive_request):
    # Full-rate drive logging was enough to destabilize teleop on the Pico.
    # Keep only sampled drive logs plus direction changes so transport logs
    # stay informative without competing with control timing.
    now_ms = utime.ticks_ms()
    last_logged_ms = state.get("last_logged_ms")
    last_direction = state.get("last_direction")
    if last_logged_ms is None or last_direction != drive_request.target_direction:
        state["last_logged_ms"] = now_ms
        state["last_direction"] = drive_request.target_direction
        return True
    if utime.ticks_diff(now_ms, last_logged_ms) >= DRIVE_LOG_SAMPLE_MS:
        state["last_logged_ms"] = now_ms
        state["last_direction"] = drive_request.target_direction
        return True
    return False


async def _send_json(ws, payload):
    await ws.send(ujson.dumps(payload))


def _command_source_id():
    return "ws-%s" % utime.ticks_ms()


def _command_type(data):
    return data.get("k")


def _command_seq(data):
    return data.get("q")


def _drive_fault_payload():
    fault = drivetrain_controller.fault_info() or {}
    return {"k": "e", "c": ERR_HARDWARE_FAULT, "m": fault.get("message", "drivetrain fault")}


def init(app):
    @app.route("/ws")
    @with_websocket
    async def ws_handler(request, ws):
        command_source_id = _command_source_id()
        drive_log_state = {"last_logged_ms": None, "last_direction": None}
        _ws_log("connected", command_source_id=command_source_id)
        try:
            while True:
                msg = await ws.receive()
                if msg is None:
                    _ws_log("client_closed", command_source_id=command_source_id)
                    break

                try:
                    data = ujson.loads(msg)
                except Exception as exc:
                    logw("Recoverable protocol error: invalid JSON (%s)" % exc)
                    await _send_json(ws, {"k": "e", "c": ERR_INVALID_JSON})
                    continue

                command_type = _command_type(data)
                seq = _command_seq(data)

                if command_type == CMD_DRIVE:
                    try:
                        drive_request = ApiDriveRequest(data)
                        if drive_request.target_rpm == 0:
                            drive_command_manager.stop(
                                command_source_id=command_source_id,
                                reason="drive_deadband_stop",
                            )
                            drivetrain_controller.request_stop(
                                seq=drive_request.seq, reason="drive_deadband_stop"
                            )
                            _ws_log(
                                "drive_deadband_stop",
                                command_source_id=command_source_id,
                                seq=drive_request.seq,
                            )
                            continue
                        drive_result = drivetrain_controller.request_drive(
                            drive_command_manager,
                            drive_request,
                            command_source_id=command_source_id,
                        )
                        if drive_result["ok"]:
                            if _should_log_drive_sample(drive_log_state, drive_request):
                                _ws_log(
                                    "drive_sample",
                                    command_source_id=command_source_id,
                                    seq=drive_request.seq,
                                    throttle=drive_request.throttle_percent,
                                    turn=drive_request.turn_percent,
                                    rpm=drive_request.target_rpm,
                                    angle=drive_request.target_angle,
                                    direction=drive_request.target_direction,
                                )
                        elif drive_result["code"] == ERR_STALE:
                            logw(
                                "Rejecting stale or foreign drive seq=%s command_source=%s"
                                % (drive_request.seq, command_source_id)
                            )
                            await _send_json(
                                ws,
                                {
                                    "k": "e",
                                    "c": ERR_STALE,
                                    "q": drive_request.seq,
                                    "m": drive_result["detail"],
                                },
                            )
                        elif drive_result["code"] == ERR_INVALID_COMMAND_SOURCE:
                            logw("Rejecting drive from non-active command source")
                            await _send_json(
                                ws,
                                {
                                    "k": "e",
                                    "c": ERR_INVALID_COMMAND_SOURCE,
                                    "q": drive_request.seq,
                                    "m": drive_result["detail"],
                                },
                            )
                        elif drive_result["code"] == ERR_RECOVERY_IN_PROGRESS:
                            logw("Rejecting drive while drivetrain recovery is in progress")
                            await _send_json(
                                ws,
                                {
                                    "k": "e",
                                    "c": ERR_RECOVERY_IN_PROGRESS,
                                    "q": drive_request.seq,
                                    "m": drive_result["detail"],
                                },
                            )
                        elif drive_result["code"] == ERR_STOP_IN_PROGRESS:
                            logw("Rejecting drive while drivetrain stop is in progress")
                            await _send_json(
                                ws,
                                {
                                    "k": "e",
                                    "c": ERR_STOP_IN_PROGRESS,
                                    "q": drive_request.seq,
                                    "m": drive_result["detail"],
                                },
                            )
                        else:
                            logw("Rejecting drive while drivetrain is faulted")
                            error_code = (
                                drive_result["code"]
                                if drive_result.get("code")
                                in (
                                    ERR_HARDWARE_FAULT,
                                    ERR_INVALID_COMMAND_SOURCE,
                                    ERR_STOP_IN_PROGRESS,
                                )
                                else ERR_HARDWARE_FAULT
                            )
                            await _send_json(
                                ws,
                                {
                                    "k": "e",
                                    "c": error_code,
                                    "q": drive_request.seq,
                                    "m": drive_result["detail"],
                                },
                            )
                    except Exception as exc:
                        loge("Recoverable drive command error: %s" % exc)
                        _print_exception(exc)
                        await _send_json(ws, {"k": "e", "c": ERR_BAD_DRIVE})
                elif command_type == CMD_STOP:
                    try:
                        did_stop = drive_command_manager.stop(command_source_id=command_source_id)
                        if did_stop:
                            result = drivetrain_controller.request_stop(
                                seq=seq, reason="manual_stop"
                            )
                            if not result["ok"]:
                                await _send_json(
                                    ws,
                                    {
                                        "k": "e",
                                        "c": result["code"],
                                        "q": seq,
                                        "m": result["detail"],
                                    },
                                )
                            elif await _wait_for_stop_completion(seq):
                                _ws_log("stop_ack", command_source_id=command_source_id, seq=seq)
                                await _send_json(ws, {"k": "a", "c": ACK_STOP, "q": seq})
                            else:
                                _ws_log(
                                    "stop_timeout",
                                    command_source_id=command_source_id,
                                    seq=seq,
                                )
                                await _send_json(
                                    ws,
                                    {
                                        "k": "e",
                                        "c": ERR_STOP_FAILED,
                                        "q": seq,
                                        "m": "stop_not_confirmed",
                                    },
                                )
                        else:
                            await _send_json(ws, {"k": "e", "c": ERR_INVALID_COMMAND_SOURCE})
                    except Exception as exc:
                        loge("Recoverable stop command error: %s" % exc)
                        _print_exception(exc)
                        await _send_json(ws, {"k": "e", "c": ERR_BAD_STOP})
                elif command_type == CMD_RECOVER:
                    try:
                        result = drivetrain_controller.request_recover(seq=seq)
                        if result["ok"]:
                            if await _wait_for_recover_completion(seq):
                                drive_command_manager.stop(reason="recovery_complete")
                                _ws_log(
                                    "recover_ack",
                                    command_source_id=command_source_id,
                                    seq=seq,
                                )
                                await _send_json(ws, {"k": "a", "c": ACK_RECOVER, "q": seq})
                            else:
                                error = drivetrain_controller.recover_error(seq) or {
                                    "detail": "recovery_failed"
                                }
                                await _send_json(
                                    ws,
                                    {
                                        "k": "e",
                                        "c": ERR_RECOVERY_FAILED,
                                        "q": seq,
                                        "m": error["detail"],
                                    },
                                )
                        else:
                            await _send_json(
                                ws,
                                {
                                    "k": "e",
                                    "c": result["code"],
                                    "q": seq,
                                    "m": result["detail"],
                                },
                            )
                    except Exception as exc:
                        loge("Recoverable recovery command error: %s" % exc)
                        _print_exception(exc)
                        await _send_json(ws, {"k": "e", "c": ERR_BAD_RECOVER, "q": seq})
                else:
                    logw("Recoverable protocol error: unknown command type=%s" % command_type)
                    await _send_json(ws, {"k": "e", "c": ERR_UNKNOWN_COMMAND})
        except Exception as exc:
            loge("WebSocket transport error: %s" % exc)
            _print_exception(exc)
        finally:
            if drive_command_manager.clear_command_source_if_active(command_source_id):
                drivetrain_controller.request_stop(reason="disconnect_stop")
                _ws_log("disconnect_stop", command_source_id=command_source_id)
            _ws_log("disconnected", command_source_id=command_source_id)

    @app.put("/drive")
    def drive(request):
        drive_request = ApiDriveRequest(request.json)
        if drive_request.target_rpm == 0:
            drive_command_manager.stop(reason="drive_deadband_stop")
            drivetrain_controller.request_stop(seq=drive_request.seq, reason="drive_deadband_stop")
            return {"status": "stopped"}
        drive_result = drivetrain_controller.request_drive(drive_command_manager, drive_request)
        if drive_result["ok"]:
            return {"status": "moving forward"}
        if drive_result["code"] == ERR_RECOVERY_IN_PROGRESS:
            return {
                "status": "rejected",
                "error": "recovery_in_progress",
                "detail": drive_result["detail"],
            }, 409
        if drive_result["code"] == ERR_STOP_IN_PROGRESS:
            return {
                "status": "rejected",
                "error": "stop_in_progress",
                "detail": drive_result["detail"],
            }, 409
        if drive_result["code"] == ERR_STALE:
            return {
                "status": "rejected",
                "error": "stale_drive",
                "detail": drive_result["detail"],
            }, 409
        return {
            "status": "faulted",
            "error": "drivetrain_fault",
            "detail": drive_result["detail"],
        }, 503

    @app.put("/stop")
    def stop(request):
        drive_command_manager.stop()
        drivetrain_controller.request_stop(reason="manual_stop")
        if _wait_for_stop_completion_blocking(None):
            return {"status": "stopped"}
        return {"status": "faulted", "error": "stop_not_confirmed"}, 503

    @app.put("/recover")
    def recover(request):
        result = drivetrain_controller.request_recover()
        if result["ok"]:
            if _wait_for_recover_completion_blocking(None):
                drive_command_manager.stop(reason="recovery_complete")
                return {"status": "recovered"}
            return {"status": "faulted", "error": "recovery_failed"}, 503
        if result["code"] == ERR_RECOVERY_CONFLICT:
            return {
                "status": "rejected",
                "error": "recovery_conflict",
                "detail": result["detail"],
            }, 409
        return {
            "status": "faulted",
            "error": "recovery_failed",
            "detail": result["detail"],
        }, 503
