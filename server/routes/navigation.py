def init(app):

    @app.route('/forward')
    def forward(request):
        # motor_control.forward()
        # state.set_speed(100)
        return {'status': 'moving forward'}

    @app.route('/stop')
    def stop(request):
        # motor_control.stop()
        # state.set_speed(0)
        return {'status': 'stopped'}