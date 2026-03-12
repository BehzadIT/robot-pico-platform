from server.routes.request_models import ApiDriveRequest
from services.robot_drive_controller import driverController
from services.robot_cross_coupled_pid import robot_pid
from lib.websockets import with_websocket
from log import *
import ujson
import utime


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
                    await _send_json(ws, {'t': 'e', 'c': 'ij'})
                    continue

                command_type = _command_type(data)
                seq = _command_seq(data)

                if command_type == "d":
                    try:
                        drive_request = ApiDriveRequest(data)
                        if not driverController.is_fresh_sequence(drive_request.seq, controller_id):
                            logw("Rejecting stale or foreign drive seq=%s controller=%s" % (drive_request.seq, controller_id))
                            await _send_json(ws, {'t': 'e', 'c': 'st', 's': drive_request.seq})
                            continue
                        driverController.drive(drive_request, controller_id=controller_id)
                        if not robot_pid.is_running():
                            _ws_log("drive_start", controller_id=controller_id, seq=drive_request.seq, rpm=drive_request.target_rpm)
                            robot_pid.start(driverController)
                    except Exception as e:
                        loge("Recoverable drive command error: %s" % e)
                        await _send_json(ws, {'t': 'e', 'c': 'bd'})
                elif command_type == "s":
                    try:
                        did_stop = driverController.stop(controller_id=controller_id)
                        if did_stop:
                            robot_pid.terminate_thread()
                            _ws_log("stop_ack", controller_id=controller_id, seq=seq)
                            await _send_json(ws, {'t': 'a', 'c': 's', 's': seq})
                        else:
                            await _send_json(ws, {'t': 'e', 'c': 'ic'})
                    except Exception as e:
                        loge("Recoverable stop command error: %s" % e)
                        await _send_json(ws, {'t': 'e', 'c': 'bs'})
                else:
                    logw("Recoverable protocol error: unknown command type=%s" % command_type)
                    await _send_json(ws, {'t': 'e', 'c': 'uc'})
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
        driverController.drive(drive_request)
        if not robot_pid.is_running():
            robot_pid.start(driverController)
        return {'status': 'moving forward'}

    @app.put('/stop')
    def stop(request):
        driverController.stop()
        robot_pid.terminate_thread()
        return {'status': 'stopped'}
