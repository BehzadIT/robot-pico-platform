from src.control.drivetrain_constants import SafetyTiming
from src.control.robot_cross_coupled_pid import robot_pid
from src.control.robot_drive_controller import driverController
from src.protocol.request_models import ApiDriveRequest
from src.support.logging import *
from lib.websockets import with_websocket
import ujson
import uasyncio as asyncio
import utime

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
ERR_INVALID_CONTROLLER = "ic"
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
        if robot_pid.is_stop_completed(seq):
            return True
        if robot_pid.stop_error(seq) is not None:
            return False
        await asyncio.sleep_ms(10)
    return robot_pid.is_stop_completed(seq)


async def _wait_for_recover_completion(seq, timeout_ms=SafetyTiming.RECOVER_COMPLETION_TIMEOUT_MS):
    deadline = utime.ticks_add(utime.ticks_ms(), timeout_ms)
    while utime.ticks_diff(deadline, utime.ticks_ms()) > 0:
        if robot_pid.is_recover_completed(seq):
            return True
        if robot_pid.recover_error(seq) is not None:
            return False
        await asyncio.sleep_ms(10)
    return robot_pid.is_recover_completed(seq)


def _wait_for_stop_completion_blocking(seq, timeout_ms=SafetyTiming.STOP_COMPLETION_TIMEOUT_MS):
    deadline = utime.ticks_add(utime.ticks_ms(), timeout_ms)
    while utime.ticks_diff(deadline, utime.ticks_ms()) > 0:
        if robot_pid.is_stop_completed(seq):
            return True
        if robot_pid.stop_error(seq) is not None:
            return False
        utime.sleep_ms(10)
    return robot_pid.is_stop_completed(seq)


def _wait_for_recover_completion_blocking(seq, timeout_ms=SafetyTiming.RECOVER_COMPLETION_TIMEOUT_MS):
    deadline = utime.ticks_add(utime.ticks_ms(), timeout_ms)
    while utime.ticks_diff(deadline, utime.ticks_ms()) > 0:
        if robot_pid.is_recover_completed(seq):
            return True
        if robot_pid.recover_error(seq) is not None:
            return False
        utime.sleep_ms(10)
    return robot_pid.is_recover_completed(seq)


def _ws_log(event, **data):
    safe_data = ", ".join(["%s=%s" % (key, data[key]) for key in data])
    logi("[ws] %s%s" % (event, (" " + safe_data) if safe_data else ""))


def _should_log_drive_sample(state, drive_request):
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


def _controller_id():
    return "ws-%s" % utime.ticks_ms()


def _command_type(data):
    return data.get("t")


def _command_seq(data):
    return data.get("s")


def _drive_fault_payload():
    fault = robot_pid.fault_info() or {}
    return {
        't': 'e',
        'c': ERR_HARDWARE_FAULT,
        'm': fault.get('message', 'drivetrain fault'),
    }


def init(app):
    @app.route('/ws')
    @with_websocket
    async def ws_handler(request, ws):
        controller_id = _controller_id()
        drive_log_state = {"last_logged_ms": None, "last_direction": None}
        _ws_log("connected", controller_id=controller_id)
        try:
            while True:
                msg = await ws.receive()
                if msg is None:
                    _ws_log("client_closed", controller_id=controller_id)
                    break

                try:
                    data = ujson.loads(msg)
                except Exception as exc:
                    logw("Recoverable protocol error: invalid JSON (%s)" % exc)
                    await _send_json(ws, {'t': 'e', 'c': ERR_INVALID_JSON})
                    continue

                command_type = _command_type(data)
                seq = _command_seq(data)

                if command_type == CMD_DRIVE:
                    try:
                        drive_request = ApiDriveRequest(data)
                        if drive_request.target_rpm == 0:
                            driverController.stop(controller_id=controller_id, reason="drive_deadband_stop")
                            robot_pid.request_stop(seq=drive_request.seq, reason="drive_deadband_stop")
                            _ws_log("drive_deadband_stop", controller_id=controller_id, seq=drive_request.seq)
                            continue
                        drive_result = robot_pid.request_drive(driverController, drive_request, controller_id=controller_id)
                        if drive_result["ok"]:
                            if _should_log_drive_sample(drive_log_state, drive_request):
                                _ws_log(
                                    "drive_sample",
                                    controller_id=controller_id,
                                    seq=drive_request.seq,
                                    rpm=drive_request.target_rpm,
                                    angle=drive_request.target_angle,
                                    direction=drive_request.target_direction,
                                )
                        elif drive_result["code"] == ERR_STALE:
                            logw("Rejecting stale or foreign drive seq=%s controller=%s" % (drive_request.seq, controller_id))
                            await _send_json(ws, {'t': 'e', 'c': ERR_STALE, 's': drive_request.seq, 'm': drive_result["detail"]})
                        elif drive_result["code"] == ERR_INVALID_CONTROLLER:
                            logw("Rejecting drive from non-active controller")
                            await _send_json(ws, {'t': 'e', 'c': ERR_INVALID_CONTROLLER, 's': drive_request.seq, 'm': drive_result["detail"]})
                        elif drive_result["code"] == ERR_RECOVERY_IN_PROGRESS:
                            logw("Rejecting drive while drivetrain recovery is in progress")
                            await _send_json(ws, {'t': 'e', 'c': ERR_RECOVERY_IN_PROGRESS, 's': drive_request.seq, 'm': drive_result["detail"]})
                        elif drive_result["code"] == ERR_STOP_IN_PROGRESS:
                            logw("Rejecting drive while drivetrain stop is in progress")
                            await _send_json(ws, {'t': 'e', 'c': ERR_STOP_IN_PROGRESS, 's': drive_request.seq, 'm': drive_result["detail"]})
                        else:
                            logw("Rejecting drive while drivetrain is faulted")
                            error_code = drive_result["code"] if drive_result.get("code") in (ERR_HARDWARE_FAULT, ERR_INVALID_CONTROLLER, ERR_STOP_IN_PROGRESS) else ERR_HARDWARE_FAULT
                            await _send_json(ws, {'t': 'e', 'c': error_code, 's': drive_request.seq, 'm': drive_result["detail"]})
                    except Exception as e:
                        loge("Recoverable drive command error: %s" % e)
                        _print_exception(e)
                        await _send_json(ws, {'t': 'e', 'c': ERR_BAD_DRIVE})
                elif command_type == CMD_STOP:
                    try:
                        did_stop = driverController.stop(controller_id=controller_id)
                        if did_stop:
                            result = robot_pid.request_stop(seq=seq, reason="manual_stop")
                            if not result["ok"]:
                                await _send_json(ws, {'t': 'e', 'c': result["code"], 's': seq, 'm': result["detail"]})
                            elif await _wait_for_stop_completion(seq):
                                _ws_log("stop_ack", controller_id=controller_id, seq=seq)
                                await _send_json(ws, {'t': 'a', 'c': ACK_STOP, 's': seq})
                            else:
                                _ws_log("stop_timeout", controller_id=controller_id, seq=seq)
                                await _send_json(ws, {'t': 'e', 'c': ERR_STOP_FAILED, 's': seq, 'm': 'stop_not_confirmed'})
                        else:
                            await _send_json(ws, {'t': 'e', 'c': ERR_INVALID_CONTROLLER})
                    except Exception as e:
                        loge("Recoverable stop command error: %s" % e)
                        _print_exception(e)
                        await _send_json(ws, {'t': 'e', 'c': ERR_BAD_STOP})
                elif command_type == CMD_RECOVER:
                    try:
                        result = robot_pid.request_recover(seq=seq)
                        if result["ok"]:
                            if await _wait_for_recover_completion(seq):
                                driverController.stop(reason="recovery_complete")
                                _ws_log("recover_ack", controller_id=controller_id, seq=seq)
                                await _send_json(ws, {'t': 'a', 'c': ACK_RECOVER, 's': seq})
                            else:
                                error = robot_pid.recover_error(seq) or {"detail": "recovery_failed"}
                                await _send_json(ws, {'t': 'e', 'c': ERR_RECOVERY_FAILED, 's': seq, 'm': error["detail"]})
                        else:
                            await _send_json(ws, {'t': 'e', 'c': result["code"], 's': seq, 'm': result["detail"]})
                    except Exception as e:
                        loge("Recoverable recovery command error: %s" % e)
                        _print_exception(e)
                        await _send_json(ws, {'t': 'e', 'c': ERR_BAD_RECOVER, 's': seq})
                else:
                    logw("Recoverable protocol error: unknown command type=%s" % command_type)
                    await _send_json(ws, {'t': 'e', 'c': ERR_UNKNOWN_COMMAND})
        except Exception as e:
            loge("WebSocket transport error: %s" % e)
            _print_exception(e)
        finally:
            if driverController.clear_controller_if_active(controller_id):
                robot_pid.request_stop(reason="disconnect_stop")
                _ws_log("disconnect_stop", controller_id=controller_id)
            _ws_log("disconnected", controller_id=controller_id)

    # Existing HTTP endpoints (unchanged)
    @app.put('/drive')
    def drive(request):
        drive_request = ApiDriveRequest(request.json)
        if drive_request.target_rpm == 0:
            driverController.stop(reason="drive_deadband_stop")
            robot_pid.request_stop(seq=drive_request.seq, reason="drive_deadband_stop")
            return {'status': 'stopped'}
        drive_result = robot_pid.request_drive(driverController, drive_request)
        if drive_result["ok"]:
            return {'status': 'moving forward'}
        if drive_result["code"] == ERR_RECOVERY_IN_PROGRESS:
            return {'status': 'rejected', 'error': 'recovery_in_progress', 'detail': drive_result["detail"]}, 409
        if drive_result["code"] == ERR_STOP_IN_PROGRESS:
            return {'status': 'rejected', 'error': 'stop_in_progress', 'detail': drive_result["detail"]}, 409
        if drive_result["code"] == ERR_STALE:
            return {'status': 'rejected', 'error': 'stale_drive', 'detail': drive_result["detail"]}, 409
        return {'status': 'faulted', 'error': 'drivetrain_fault', 'detail': drive_result["detail"]}, 503

    @app.put('/stop')
    def stop(request):
        driverController.stop()
        robot_pid.request_stop(reason="manual_stop")
        if _wait_for_stop_completion_blocking(None):
            return {'status': 'stopped'}
        return {'status': 'faulted', 'error': 'stop_not_confirmed'}, 503

    @app.put('/recover')
    def recover(request):
        result = robot_pid.request_recover()
        if result["ok"]:
            if _wait_for_recover_completion_blocking(None):
                driverController.stop(reason="recovery_complete")
                return {'status': 'recovered'}
            return {'status': 'faulted', 'error': 'recovery_failed'}, 503
        if result["code"] == ERR_RECOVERY_CONFLICT:
            return {'status': 'rejected', 'error': 'recovery_conflict', 'detail': result["detail"]}, 409
        return {'status': 'faulted', 'error': 'recovery_failed', 'detail': result["detail"]}, 503
