from server.routes.request_models import ApiDriveRequest
from services.robot_drive_controller import driverController
from services.robot_cross_coupled_pid import robot_pid
from lib.websockets import with_websocket
from log import *
import ujson
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
ERR_RECOVERY_CONFLICT = "rc"
ERR_RECOVERY_FAILED = "rf"


def _ws_log(event, **data):
    safe_data = ", ".join(["%s=%s" % (key, data[key]) for key in data])
    logi("[ws] %s%s" % (event, (" " + safe_data) if safe_data else ""))


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
                        if not driverController.is_fresh_sequence(drive_request.seq, controller_id):
                            logw("Rejecting stale or foreign drive seq=%s controller=%s" % (drive_request.seq, controller_id))
                            await _send_json(ws, {'t': 'e', 'c': ERR_STALE, 's': drive_request.seq})
                            continue
                        start_result = robot_pid.start(driverController)
                        if start_result["ok"]:
                            driverController.drive(drive_request, controller_id=controller_id)
                            _ws_log(
                                "drive_start",
                                controller_id=controller_id,
                                seq=drive_request.seq,
                                rpm=drive_request.target_rpm,
                                angle=drive_request.target_angle,
                            )
                        elif start_result["code"] == ERR_RECOVERY_IN_PROGRESS:
                            logw("Rejecting drive while drivetrain recovery is in progress")
                            await _send_json(ws, {'t': 'e', 'c': ERR_RECOVERY_IN_PROGRESS, 's': drive_request.seq, 'm': start_result["detail"]})
                        else:
                            logw("Rejecting drive while drivetrain is faulted")
                            await _send_json(ws, {'t': 'e', 'c': ERR_HARDWARE_FAULT, 's': drive_request.seq, 'm': start_result["detail"]})
                    except Exception as e:
                        loge("Recoverable drive command error: %s" % e)
                        await _send_json(ws, {'t': 'e', 'c': ERR_BAD_DRIVE})
                elif command_type == CMD_STOP:
                    try:
                        did_stop = driverController.stop(controller_id=controller_id)
                        if did_stop:
                            robot_pid.terminate_thread()
                            _ws_log("stop_ack", controller_id=controller_id, seq=seq)
                            await _send_json(ws, {'t': 'a', 'c': ACK_STOP, 's': seq})
                        else:
                            await _send_json(ws, {'t': 'e', 'c': ERR_INVALID_CONTROLLER})
                    except Exception as e:
                        loge("Recoverable stop command error: %s" % e)
                        await _send_json(ws, {'t': 'e', 'c': ERR_BAD_STOP})
                elif command_type == CMD_RECOVER:
                    try:
                        result = robot_pid.recover()
                        if result["ok"]:
                            driverController.stop(reason="recovery_complete")
                            _ws_log("recover_ack", controller_id=controller_id, seq=seq)
                            await _send_json(ws, {'t': 'a', 'c': ACK_RECOVER, 's': seq})
                        else:
                            await _send_json(ws, {'t': 'e', 'c': result["code"], 's': seq, 'm': result["detail"]})
                    except Exception as e:
                        loge("Recoverable recovery command error: %s" % e)
                        await _send_json(ws, {'t': 'e', 'c': ERR_BAD_RECOVER, 's': seq})
                else:
                    logw("Recoverable protocol error: unknown command type=%s" % command_type)
                    await _send_json(ws, {'t': 'e', 'c': ERR_UNKNOWN_COMMAND})
        except Exception as e:
            loge("WebSocket transport error: %s" % e)
        finally:
            if driverController.clear_controller_if_active(controller_id):
                robot_pid.terminate_thread()
                _ws_log("disconnect_stop", controller_id=controller_id)
            _ws_log("disconnected", controller_id=controller_id)

    # Existing HTTP endpoints (unchanged)
    @app.put('/drive')
    def drive(request):
        drive_request = ApiDriveRequest(request.json)
        start_result = robot_pid.start(driverController)
        if start_result["ok"]:
            driverController.drive(drive_request)
            return {'status': 'moving forward'}
        if start_result["code"] == ERR_RECOVERY_IN_PROGRESS:
            return {'status': 'rejected', 'error': 'recovery_in_progress', 'detail': start_result["detail"]}, 409
        return {'status': 'faulted', 'error': 'drivetrain_fault', 'detail': start_result["detail"]}, 503

    @app.put('/stop')
    def stop(request):
        driverController.stop()
        robot_pid.terminate_thread()
        return {'status': 'stopped'}

    @app.put('/recover')
    def recover(request):
        result = robot_pid.recover()
        if result["ok"]:
            driverController.stop(reason="recovery_complete")
            return {'status': 'recovered'}
        if result["code"] == ERR_RECOVERY_CONFLICT:
            return {'status': 'rejected', 'error': 'recovery_conflict', 'detail': result["detail"]}, 409
        return {'status': 'faulted', 'error': 'recovery_failed', 'detail': result["detail"]}, 503
