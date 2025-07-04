class ApiDriveRequest:
    def __init__(self, data):
        # data is a dict, e.g. from ujson.loads()
        self.target_rpm = int(data.get('target_rpm', 0))
        self.target_direction = int(data.get('target_direction', 1))
        # Add more fields as needed
