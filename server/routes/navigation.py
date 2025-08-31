from server.routes.request_models import ApiDriveRequest
from services.robot_drive_controller import driverController
from lib.websockets import with_websocket
import ujson

def init(app):
    @app.route('/ws')
    @with_websocket
    async def ws_handler(request, ws):
        print("WebSocket client connected")
        try:
            while True:
                msg = await ws.receive()
                if msg is None:
                    break  # Client disconnected

                print('Received via WebSocket:', msg)
                print('Type of msg:', type(msg))
                try:
                    data = ujson.loads(msg)
                except Exception:
                    await ws.send(ujson.dumps({'error': 'invalid_json'}))
                    continue
                print('type of drive:', data.get("type"))

                # Handle 'drive' command
                if data.get("type") == "drive":
                    try:
                        drive_request = ApiDriveRequest(data)
                        driverController.drive(drive_request)
                        await ws.send(ujson.dumps({'status': 'moving forward'}))
                    except Exception as e:
                        await ws.send(ujson.dumps({'error': f'bad_drive_command: {str(e)}'}))
                # Handle 'stop' command
                elif data.get("type") == "stop":
                    try:
                        driverController.stop()
                        await ws.send(ujson.dumps({'status': 'stopped'}))
                    except Exception as e:
                        await ws.send(ujson.dumps({'error': f'bad_stop_command: {str(e)}'}))
                else:
                    await ws.send(ujson.dumps({'error': 'unknown_command'}))
        except Exception as e:
            print('WebSocket error:', e)
        print("WebSocket client disconnected")

    # Existing HTTP endpoints (unchanged)
    @app.put('/drive')
    def drive(request):
        drive_request = ApiDriveRequest(request.json)
        driverController.drive(drive_request)
        return {'status': 'moving forward'}

    @app.put('/stop')
    def stop(request):
        driverController.stop()
        return {'status': 'stopped'}
