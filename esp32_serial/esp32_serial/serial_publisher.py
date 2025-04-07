import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32MultiArray

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__("serial_reader")

        # Define Serial Port and Baud Rate
        self.serial_port = "/dev/ttyUSB2"  # Change this based on your device
        self.baud_rate = 115200  

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            return

        # Create ROS 2 publisher (publishes an array of two floats)
        self.publisher_ = self.create_publisher(Float32MultiArray, "esp32_sensor_data", 10)

        # Timer to read serial data
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            try:
                # Read and decode serial data
                line = self.ser.readline().decode("utf-8").strip()
                
                # Split the string into two float values
                parts = line.split(",")  # Expecting "left_value,right_value"
                if len(parts) == 2:
                    left_sensor = float(parts[0])
                    right_sensor = float(parts[1])

                    # Publish as Float32MultiArray
                    msg = Float32MultiArray()
                    msg.data = [left_sensor, right_sensor]
                    self.publisher_.publish(msg)

                    self.get_logger().info(f"Published: Left = {left_sensor}, Right = {right_sensor}")
                else:
                    self.get_logger().warn(f"Invalid data format: {line}")

            except ValueError:
                self.get_logger().error(f"Failed to convert data to float: {line}")
            except Exception as e:
                self.get_logger().error(f"Error reading serial data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()







# import rclpy
# from rclpy.node import Node
# import serial
# from std_msgs.msg import String

# class SerialReaderNode(Node):
#     def __init__(self):
#         super().__init__("serial_reader")

#         # Define Serial Port and Baud Rate
#         self.serial_port = "/dev/ttyUSB0"  # Change this based on your device
#         self.baud_rate = 115200  

#         try:
#             self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
#             self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
#         except serial.SerialException as e:
#             self.get_logger().error(f"Could not open serial port: {e}")
#             return

#         # Create ROS 2 publisher
#         self.publisher_ = self.create_publisher(String, "esp32_serial_data", 10)

#         # Timer to read serial data
#         self.timer = self.create_timer(0.05, self.read_serial_data)

#     def read_serial_data(self):
#         if self.ser.in_waiting > 0:
#             try:
#                 line = self.ser.readline().decode("utf-8").strip()
#                 if line:
#                     msg = String()
#                     msg.data = line
#                     self.publisher_.publish(msg)
#                     self.get_logger().info(f"Published: {line}")
#             except Exception as e:
#                 self.get_logger().error(f"Error reading serial data: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialReaderNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Node stopped by user")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()
