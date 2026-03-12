class ApiDriveRequest:
    def __init__(self, data):
        self.target_rpm = int(data.get('r', data.get('target_rpm', 0)))
        self.target_direction = int(data.get('d', data.get('target_direction', 1)))
        self.seq = self._to_optional_int(data.get('s', data.get('seq')))
        self.sent_at_ms = self._to_optional_int(data.get('m', data.get('sent_at_ms')))

    @staticmethod
    def _to_optional_int(value):
        if value is None:
            return None
        return int(value)
