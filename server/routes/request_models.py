class ApiDriveRequest:
    def __init__(self, data):
        # data is a dict, e.g. from ujson.loads()
        self.target_rpm = int(data.get('target_rpm', 0))
        self.target_direction = int(data.get('target_direction', 1))
        self.seq = self._to_optional_int(data.get('seq'))
        self.sent_at_ms = self._to_optional_int(data.get('sent_at_ms'))

    @staticmethod
    def _to_optional_int(value):
        if value is None:
            return None
        return int(value)
