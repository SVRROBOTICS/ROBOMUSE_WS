class MoonServoError(Exception):
    """Base class for Moon Servo Motor exceptions."""
    pass

class ConnectionError(MoonServoError):
    """Raised when Modbus connection fails."""
    pass
