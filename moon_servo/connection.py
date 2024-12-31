from pymodbus.client import ModbusSerialClient

class ModbusConnection:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=5, parity='N', stopbits=1, bytesize=8):
        self.client = ModbusSerialClient(
            port=port, baudrate=baudrate, timeout=timeout, parity=parity, stopbits=stopbits, bytesize=bytesize
        )

    def connect(self):
        if self.client.connect():
            print("Modbus Connection Successful")
            return True
        else:
            raise ConnectionError("Modbus Connection Failed")

    def disconnect(self):
        self.client.close()
        print("Modbus Connection Closed")

    def get_client(self):
        return self.client
