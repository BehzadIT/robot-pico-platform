from src.transport import mobile_app_transport


def register_routes(app):
    mobile_app_transport.init(app)
