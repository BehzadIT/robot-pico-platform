from server.routes.request_models import ApiDriveRequest
from services.robot_drive_controller import driverController

def init(app):

    @app.put('/drive')
    def drive(request):
        drive_request = ApiDriveRequest(request.json)
        driverController.drive(drive_request)

        # motor_control.forward()
        # state.set_speed(100)
        return {'status': 'moving forward'}

    @app.put('/stop')
    def stop(request):
        driverController.stop()
        # motor_control.stop()
        # state.set_speed(0)
        return {'status': 'stopped'}