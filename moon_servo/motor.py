import time
from .connection import ModbusConnection

class MoonServoMotor:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, base_address: int = 0):
        connection: ModbusConnection = ModbusConnection(port=port, baudrate=baudrate, timeout=5, parity='N', stopbits=1, bytesize=8)
        self.client = connection.get_client()
        self.base_address = base_address

    def connect(self):
        if self.client.connect():
            print("Modbus Connection Successful")
            return True
        else:
            raise ConnectionError("Modbus Connection Failed")

    def disconnect(self):
        self.client.close()
        print("Modbus Connection Closed")

    def enable_driver(self):
        command_register1 = self.base_address + 124
        command_register2 = self.base_address + 1124
        opcode = 0x009F
        self._write_register1(command_register1, opcode, "Enable driver1")
        self._write_register1(command_register2, opcode, "Enable driver2")

    def disable_driver(self):
        command_register1 = self.base_address + 124
        command_register2 = self.base_address + 1124
        opcode = 0x009E
        self._write_register1(command_register1, opcode, "Disable driver1")
        self._write_register1(command_register2, opcode, "Disable driver2")

    def start_jogging(self):
        command_register1 = self.base_address + 124
        command_register2 = self.base_address + 1124
        opcode = 0x0096
        self._write_register1(command_register1, opcode, "Start jogging1")
        self._write_register1(command_register2, opcode, "Start jogging2")

    def stop_jogging(self):
        command_register1 = self.base_address + 124
        command_register2 = self.base_address + 1124
        opcode = 0x00D8
        self._write_register1(command_register1, opcode, "Stop jogging1")
        self._write_register1(command_register2, opcode, "Stop jogging2")

    def set_speed(self, speed_value1,speed_value2, run_time=5):
        speed_register1 = self.base_address + 342
        speed_register2 = self.base_address + 1342
        self._write_32bit_register1(speed_register1, speed_value1, "Set speed1")
        self._write_32bit_register1(speed_register2, speed_value2, "Set speed2")
        time.sleep(run_time)
        self._write_32bit_register1(speed_register1, 0, "Stop motor after timeout")
        self._write_32bit_register1(speed_register2, 0, "Stop motor after timeout")

    def set_acceleration(self, accel_value):
        accel_register1 = self.base_address + 338
        accel_register2 = self.base_address + 1338
        self._write_32bit_register1(accel_register1, accel_value, "Set acceleration1")
        self._write_32bit_register1(accel_register2, accel_value, "Set acceleration2")

    def set_deceleration(self, decel_value):
        decel_register1 = self.base_address + 340
        decel_register2 = self.base_address + 1340
        self._write_32bit_register1(decel_register1, decel_value, "Set deceleration1")
        self._write_32bit_register1(decel_register2, decel_value, "Set deceleration2")

    def _write_register1(self, register, value, action):
        try:
            self.client.write_register(register, value)
            print(f"{action} successful.")
        except Exception as e:
            print(f"Error during {action}: {e}")
            raise  

    def _write_32bit_register1(self, register, value, action):
        try:
            high_word = (value >> 16) & 0xFFFF
            low_word = value & 0xFFFF
            self.client.write_registers(register, [high_word, low_word])
            print(f"{action} successful.")
        except Exception as e:
            print(f"Error during {action}: {e}")
            raise

    def read_register_32bit1(self,register_address, endian='big'):
        """
        Reads a 32-bit value from a Modbus register.
        
        :param register_address: The starting address of the register (Modbus address).
        :param endian: The byte order, 'big' or 'little'. Defaults to 'big'.
        :return: The 32-bit integer value, or None if an error occurs.
        """
        zero_based_address = register_address - 40001  # Convert to zero-based address
        
        try:
            # Read two 16-bit registers
            result = self.client.read_holding_registers(zero_based_address, 2)
            
            if result.isError():
                print(f"Error reading register {register_address}, error: {result}")
                return None
            
            # Combine the two 16-bit registers into a 32-bit integer
            high, low = result.registers
            if endian == 'big':
                value = (high << 16) | low  # Big-endian: High word first
            elif endian == 'little':
                value = (low << 16) | high  # Little-endian: Low word first
            else:
                raise ValueError("Invalid endian type. Use 'big' or 'little'.")
            
            print(f"Value at register {register_address}: {value}")
            return value
        
        except Exception as e:
            print(f"Error reading register {register_address}: {e}")
            return None

    def read_register_32bit2(self,register_address, endian='big'):
        """
        Reads a 32-bit value from a Modbus register.
        
        :param register_address: The starting address of the register (Modbus address).
        :param endian: The byte order, 'big' or 'little'. Defaults to 'big'.
        :return: The 32-bit integer value, or None if an error occurs.
        """
        zero_based_address = register_address - 40001  # Convert to zero-based address
        
        try:
            # Read two 16-bit registers
            result = self.client.read_holding_registers(zero_based_address, 2)
            
            if result.isError():
                print(f"Error reading register {register_address}, error: {result}")
                return None
            
            # Combine the two 16-bit registers into a 32-bit integer
            high, low = result.registers
            if endian == 'big':
                value = (high << 16) | low  # Big-endian: High word first
            elif endian == 'little':
                value = (low << 16) | high  # Little-endian: Low word first
            else:
                raise ValueError("Invalid endian type. Use 'big' or 'little'.")
            
            print(f"Value at register {register_address}: {value}")
            return value
        
        except Exception as e:
            print(f"Error reading register {register_address}: {e}")
            return None    
            