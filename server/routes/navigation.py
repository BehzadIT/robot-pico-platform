from services.robot_cross_coupled_pid import main_control_loop
import _thread

def init(app):

    @app.route('/forward')
    def forward(request):
        _thread.start_new_thread(main_control_loop, ())
        # motor_control.forward()
        # state.set_speed(100)
        return {'status': 'moving forward'}

    @app.route('/stop')
    def stop(request):
        # motor_control.stop()
        # state.set_speed(0)
        return {'status': 'stopped'}