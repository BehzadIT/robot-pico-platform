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

                logd("Received via WebSocket: %s" % msg)
                try:
                    data = ujson.loads(msg)
                except Exception as exc:
                    logw("Recoverable protocol error: invalid JSON (%s)" % exc)
                    await _send_json(ws, {'error': 'invalid_json'})
                    continue

                command_type = data.get("type")
                seq = data.get("seq")
                logd("Command type=%s seq=%s controller=%s" % (command_type, seq, controller_id))

                if command_type == "drive":
                    try:
                        drive_request = ApiDriveRequest(data)
                        if not driverController.is_fresh_sequence(drive_request.seq, controller_id):
                            logw("Rejecting stale or foreign drive seq=%s controller=%s" % (drive_request.seq, controller_id))
                            await _send_json(ws, {'error': 'stale_command', 'seq': drive_request.seq})
                            continue
                        driverController.drive(drive_request, controller_id=controller_id)
                        if not robot_pid.is_running():
                            robot_pid.start(driverController)
                    except Exception as e:
                        loge("Recoverable drive command error: %s" % e)
                        await _send_json(ws, {'error': 'bad_drive_command', 'detail': str(e)})
                elif command_type == "stop":
                    try:
                        did_stop = driverController.stop(controller_id=controller_id)
                        if did_stop:
                            robot_pid.terminate_thread()
                            await _send_json(ws, {'status': 'stopped', 'type': 'stop_ack'})
                        else:
                            await _send_json(ws, {'error': 'inactive_controller'})
                    except Exception as e:
                        loge("Recoverable stop command error: %s" % e)
                        await _send_json(ws, {'error': 'bad_stop_command', 'detail': str(e)})
                else:
                    logw("Recoverable protocol error: unknown command type=%s" % command_type)
                    await _send_json(ws, {'error': 'unknown_command'})
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
