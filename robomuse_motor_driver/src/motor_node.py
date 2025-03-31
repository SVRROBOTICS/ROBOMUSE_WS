#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from moon_servo import MoonServoMotor  # Replace with actual module/class from moon_servo
import math
from rclpy.clock import Clock


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Robot parameters
        self.wheel_diameter = 0.125  # 125 mm = 0.125 m
        self.wheel_base = 0.53  # Distance between the two wheels
        self.gear_ratio = 20   # 1:20 Gear Ratio
        self.encoder_ppr = 10000  # Computed encoder resolution considering gearbox ratio
        self.encoder_resolution = (self.encoder_ppr * self.gear_ratio)/(math.pi * self.wheel_diameter)
        # self.encoder_resolution = 502400.00

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = Clock().now()  

        # Previous encoder values
        self.prev_left_ticks = 0.0
        self.prev_right_ticks = 0.0

        # Motor initialization
        self.motor_driver = MoonServoMotor(port='/dev/motor_driver', baudrate=9600, base_address=0)
        self.motor_driver.connect()
        self.motor_driver.enable_driver1()
        self.motor_driver.enable_driver2()
        self.motor_driver.reset_encoder1()
        self.motor_driver.reset_encoder2()
        self.motor_driver.set_acceleration1(300)
        self.motor_driver.set_deceleration1(300)
        self.motor_driver.set_acceleration2(300)
        self.motor_driver.set_deceleration2(300)
        self.motor_driver.start_jogging1()
        self.motor_driver.start_jogging2()
        self.cmd_vel_callback(Twist())

        # ROS publishers
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("MotorControllerNode has been started.")

        # Timer for publishing odometry
        self.create_timer(1/40, self.publish_odometry)  # 40 Hz

    def cmd_vel_callback(self, msg):
        try:
            linear_velocity = msg.linear.x
            angular_velocity = msg.angular.z

            left_motor_speed, right_motor_speed = self.calculate_motor_speeds(linear_velocity, angular_velocity)

            self.motor_driver.set_speed1(left_motor_speed)
            self.motor_driver.set_speed2(right_motor_speed)

            self.get_logger().info(f"Set motor speeds: Left={left_motor_speed}, Right={right_motor_speed}")
        except Exception as e:
            self.get_logger().error(f"Error in cmd_vel_callback: {e}")

    def calculate_motor_speeds(self, linear, angular):
        left_motor_speed = linear - (angular * self.wheel_base / 2.0)
        right_motor_speed = linear + (angular * self.wheel_base / 2.0)
        return left_motor_speed, right_motor_speed

    def publish_odometry(self):

        # Get encoder ticks
        left_ticks = float(self.motor_driver.get_encoder1() or self.prev_left_ticks)
        right_ticks = float(self.motor_driver.get_encoder2() or self.prev_right_ticks)

        # Log encoder values
        self.get_logger().info(f"Left encoder ticks: {left_ticks}, Right encoder ticks: {right_ticks}")

        # Compute tick differences (ensure float calculations)
        delta_left_ticks = float(left_ticks - self.prev_left_ticks)
        delta_right_ticks = float(right_ticks - self.prev_right_ticks)

        # Store previous values
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        # Convert ticks to distances (meters)
        left_distance = delta_left_ticks / self.encoder_resolution
        right_distance = delta_right_ticks / self.encoder_resolution

        # Compute change in pose
        delta_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base  # Corrected formula
        self.theta += delta_theta

        # Update pose
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)


        # Log differences
        self.get_logger().info(f"Delta Distance: {delta_distance}, Theta: {self.theta}")

        # Create odometry message
        odom = Odometry()
        current_time = Clock().now()  
        delta_time = (current_time - self.last_time).nanoseconds / 1e9
        if delta_time == 0:  # Prevent division by zero
            return
        self.last_time = current_time
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # Convert theta (yaw) to quaternion
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.theta)

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = delta_distance / delta_time
        odom.twist.twist.angular.z = delta_theta / delta_time

        # Publish odometry
        self.odom_publisher.publish(odom)

        # Publish transform
        transform = TransformStamped()
        current_time = Clock().now()  
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y

        # Use the same quaternion for the TF transform
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)

  
        
    def shutdown_motors(self):
        try:
            self.motor_driver.stop_jogging1()
            self.motor_driver.stop_jogging2()
            self.motor_driver.reset_encoder1()
            self.motor_driver.reset_encoder2()
            self.motor_driver.disable_driver1()
            self.motor_driver.disable_driver2()
            self.get_logger().info("Motors stopped and drivers disabled successfully.")
        except Exception as e:
            self.get_logger().error(f"Error during motor shutdown: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node is shutting down.")
    finally:
        node.shutdown_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()














# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String  # Import for ESP32 data topic
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped
# from tf_transformations import quaternion_from_euler
# from moon_servo import MoonServoMotor  # Replace with actual module/class from moon_servo
# import math
# from rclpy.clock import Clock

# from std_msgs.msg import Float32MultiArray
# import re  # Regular expressions for parsing sensor data

# _SAFE_DIST = 45.0

# class MotorControllerNode(Node):
#     def __init__(self):
#         super().__init__('motor_controller')

#         # Motor initialization
#         self.motor_driver = MoonServoMotor(port='/dev/motor_driver', baudrate=9600, base_address=0)
#         self.motor_driver.connect()
#         self.motor_driver.enable_driver1()
#         self.motor_driver.enable_driver2()
#         self.motor_driver.reset_encoder1()
#         self.motor_driver.reset_encoder2()
#         self.motor_driver.set_acceleration1(300)
#         self.motor_driver.set_deceleration1(300)
#         self.motor_driver.set_acceleration2(300)
#         self.motor_driver.set_deceleration2(300)
#         self.motor_driver.start_jogging1()
#         self.motor_driver.start_jogging2()

#         # Robot parameters
#         self.wheel_diameter = 0.125  # 125 mm = 0.125 m
#         self.wheel_base = 0.53  # Distance between the two wheels
#         self.gear_ratio = 20   # 1:20 Gear Ratio
#         self.encoder_ppr = 10000  # Computed encoder resolution considering gearbox ratio
#         self.encoder_resolution = (self.encoder_ppr * self.gear_ratio) / (math.pi * self.wheel_diameter)

#         # Odometry state
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.last_time = Clock().now()  

#         # Previous encoder values
#         self.prev_left_ticks = 0.0
#         self.prev_right_ticks = 0.0

#         # Sensor readings
#         self.sensor1 = 0.0
#         self.sensor2 = 0.0

#         #Distance sensors
#         self.left_distance_sensor = 0.0
#         self.right_distance_sensor = 0.0

#         # ROS publishers
#         self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Subscribe to cmd_vel topic
#         self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

#         # Subscribe to ESP32 sensor data
#         self.sensor_subscription = self.create_subscription(Float32MultiArray, '/esp32_sensor_data', self.esp32_callback, 10)

#         # Timer for publishing odometry
#         self.create_timer(1/40, self.publish_odometry)  # 40 Hz

#         self.get_logger().info("MotorControllerNode has been started.")

#     def esp32_callback(self, msg):
#         """ Callback function to handle ESP32 sensor data. """
#         try:
#             if len(msg.data) >= 2:
#                 self.left_distance_sensor = msg.data[0]  # First value
#                 self.right_distance_sensor = msg.data[1]  # Second value
#                 self.get_logger().info(f'Distance sensor: Left={self.left_distance_sensor}, Right={self.right_distance_sensor}')
#                 if not self.check_safety():
#                     self.get_logger().info(f"Stopping robot. Distance sensor detected object within {_SAFE_DIST}")
#                     left_motor_speed = right_motor_speed = 0.0
#                     self.motor_driver.set_speed1(left_motor_speed)
#                     self.motor_driver.set_speed2(right_motor_speed)
#             else:
#                 self.get_logger().warn("Received data array has fewer than 2 elements.")
#         except Exception as e:
#             self.get_logger().error(f"Error parsing ESP32 data: {e}")

#     def check_safety(self):
#         if self.left_distance_sensor < _SAFE_DIST and self.right_distance_sensor < _SAFE_DIST:
#             return False
#         else:
#             return True

#     def cmd_vel_callback(self, msg):
#         """ Handles velocity commands and sets motor speeds. """
#         try:
#             if self.check_safety():
#                 linear_velocity = msg.linear.x
#                 angular_velocity = msg.angular.z
#                 left_motor_speed, right_motor_speed = self.calculate_motor_speeds(linear_velocity, angular_velocity)
#             else:
#                 self.get_logger().info(f"Stopping robot. Distance sensor detected object within {_SAFE_DIST}")
#                 left_motor_speed = right_motor_speed = 0.0
#             self.motor_driver.set_speed1(left_motor_speed)
#             self.motor_driver.set_speed2(right_motor_speed)
#             self.get_logger().info(f"Set motor speeds: Left={left_motor_speed}, Right={right_motor_speed}")
#         except Exception as e:
#             self.get_logger().error(f"Error in cmd_vel_callback: {e}")

#     def calculate_motor_speeds(self, linear, angular):
#         left_motor_speed = linear - (angular * self.wheel_base / 2.0)
#         right_motor_speed = linear + (angular * self.wheel_base / 2.0)
#         return left_motor_speed, right_motor_speed

#     def publish_odometry(self):
#         """ Computes and publishes odometry data. """
#         left_ticks = float(self.motor_driver.get_encoder1() or self.prev_left_ticks)
#         right_ticks = float(self.motor_driver.get_encoder2() or self.prev_right_ticks)

#         self.get_logger().debug(f"Left encoder ticks: {left_ticks}, Right encoder ticks: {right_ticks}")

#         delta_left_ticks = float(left_ticks - self.prev_left_ticks)
#         delta_right_ticks = float(right_ticks - self.prev_right_ticks)

#         self.prev_left_ticks = left_ticks
#         self.prev_right_ticks = right_ticks

#         left_distance = delta_left_ticks / self.encoder_resolution
#         right_distance = delta_right_ticks / self.encoder_resolution

#         delta_distance = (left_distance + right_distance) / 2.0
#         delta_theta = (right_distance - left_distance) / self.wheel_base
#         self.theta += delta_theta

#         self.x += delta_distance * math.cos(self.theta)
#         self.y += delta_distance * math.sin(self.theta)

#         self.get_logger().info(f"Delta Distance: {delta_distance}, Theta: {self.theta}")

#         odom = Odometry()
#         current_time = Clock().now()  
#         delta_time = (current_time - self.last_time).nanoseconds / 1e9
#         if delta_time == 0:
#             return
#         self.last_time = current_time
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_link'
#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y

#         qx, qy, qz, qw = quaternion_from_euler(0, 0, self.theta)
#         odom.pose.pose.orientation.x = qx
#         odom.pose.pose.orientation.y = qy
#         odom.pose.pose.orientation.z = qz
#         odom.pose.pose.orientation.w = qw

#         odom.twist.twist.linear.x = delta_distance / delta_time
#         odom.twist.twist.angular.z = delta_theta / delta_time

#         self.odom_publisher.publish(odom)

#         transform = TransformStamped()
#         transform.header.stamp = current_time.to_msg()
#         transform.header.frame_id = 'odom'
#         transform.child_frame_id = 'base_link'
#         transform.transform.translation.x = self.x
#         transform.transform.translation.y = self.y

#         transform.transform.rotation.x = qx
#         transform.transform.rotation.y = qy
#         transform.transform.rotation.z = qz
#         transform.transform.rotation.w = qw

#         self.tf_broadcaster.sendTransform(transform)

#     def shutdown_motors(self):
#         """ Stops motors and disables drivers on shutdown. """
#         try:
#             self.motor_driver.stop_jogging1()
#             self.motor_driver.stop_jogging2()
#             self.motor_driver.reset_encoder1()
#             self.motor_driver.reset_encoder2()
#             self.motor_driver.disable_driver1()
#             self.motor_driver.disable_driver2()
#             self.get_logger().info("Motors stopped and drivers disabled successfully.")
#         except Exception as e:
#             self.get_logger().error(f"Error during motor shutdown: {e}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = MotorControllerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Node is shutting down.")
#     finally:
#         node.shutdown_motors()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()










# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped
# from tf_transformations import quaternion_from_euler
# from moon_servo import MoonServoMotor  # Replace with actual module/class from moon_servo
# import math
# from rclpy.clock import Clock


# class MotorControllerNode(Node):
#     def __init__(self):
#         super().__init__('motor_controller')

#         # Motor initialization
#         self.motor_driver = MoonServoMotor(port='/dev/motor_driver', baudrate=9600, base_address=0)
#         self.motor_driver.connect()
#         self.motor_driver.enable_driver1()
#         self.motor_driver.enable_driver2()
#         self.motor_driver.reset_encoder1()
#         self.motor_driver.reset_encoder2()
#         self.motor_driver.set_acceleration1(300)
#         self.motor_driver.set_deceleration1(300)
#         self.motor_driver.set_acceleration2(300)
#         self.motor_driver.set_deceleration2(300)
#         self.motor_driver.start_jogging1()
#         self.motor_driver.start_jogging2()

#         # Robot parameters
#         self.wheel_diameter = 0.125  # 125 mm = 0.125 m
#         self.wheel_base = 0.53  # Distance between the two wheels
#         self.gear_ratio = 20   # 1:20 Gear Ratio
#         self.encoder_ppr = 10000  # Computed encoder resolution considering gearbox ratio
#         self.encoder_resolution = (self.encoder_ppr * self.gear_ratio)/(math.pi * self.wheel_diameter)
#         # self.encoder_resolution = 502400.00

#         # Odometry state
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.last_time = Clock().now()  

#         # Previous encoder values
#         self.prev_left_ticks = 0.0
#         self.prev_right_ticks = 0.0

#         # ROS publishers
#         self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Subscribe to cmd_vel topic
#         self.subscription = self.create_subscription(
#             Twist,
#             'cmd_vel',
#             self.cmd_vel_callback,
#             10
#         )
#         self.get_logger().info("MotorControllerNode has been started.")

#         # Timer for publishing odometry
#         self.create_timer(1/40, self.publish_odometry)  # 40 Hz

#     def cmd_vel_callback(self, msg):
#         try:
#             linear_velocity = msg.linear.x
#             angular_velocity = msg.angular.z

#             left_motor_speed, right_motor_speed = self.calculate_motor_speeds(linear_velocity, angular_velocity)

#             self.motor_driver.set_speed1(left_motor_speed)
#             self.motor_driver.set_speed2(right_motor_speed)

#             self.get_logger().info(f"Set motor speeds: Left={left_motor_speed}, Right={right_motor_speed}")
#         except Exception as e:
#             self.get_logger().error(f"Error in cmd_vel_callback: {e}")

#     def calculate_motor_speeds(self, linear, angular):
#         left_motor_speed = linear - (angular * self.wheel_base / 2.0)
#         right_motor_speed = linear + (angular * self.wheel_base / 2.0)
#         return left_motor_speed, right_motor_speed

#     def publish_odometry(self):

#         # Get encoder ticks
#         left_ticks = float(self.motor_driver.get_encoder1() or self.prev_left_ticks)
#         right_ticks = float(self.motor_driver.get_encoder2() or self.prev_right_ticks)

#         # Log encoder values
#         self.get_logger().info(f"Left encoder ticks: {left_ticks}, Right encoder ticks: {right_ticks}")

#         # Compute tick differences (ensure float calculations)
#         delta_left_ticks = float(left_ticks - self.prev_left_ticks)
#         delta_right_ticks = float(right_ticks - self.prev_right_ticks)

#         # Store previous values
#         self.prev_left_ticks = left_ticks
#         self.prev_right_ticks = right_ticks

#         # Convert ticks to distances (meters)
#         left_distance = delta_left_ticks / self.encoder_resolution
#         right_distance = delta_right_ticks / self.encoder_resolution

#         # Compute change in pose
#         delta_distance = (left_distance + right_distance) / 2.0
#         delta_theta = (right_distance - left_distance) / self.wheel_base  # Corrected formula
#         self.theta += delta_theta

#         # Update pose
#         self.x += delta_distance * math.cos(self.theta)
#         self.y += delta_distance * math.sin(self.theta)


#         # Log differences
#         self.get_logger().info(f"Delta Distance: {delta_distance}, Theta: {self.theta}")

#         # Create odometry message
#         odom = Odometry()
#         current_time = Clock().now()  
#         delta_time = (current_time - self.last_time).nanoseconds / 1e9
#         if delta_time == 0:  # Prevent division by zero
#             return
#         self.last_time = current_time
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_link'
#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y

#         # Convert theta (yaw) to quaternion
#         qx, qy, qz, qw = quaternion_from_euler(0, 0, self.theta)

#         odom.pose.pose.orientation.x = qx
#         odom.pose.pose.orientation.y = qy
#         odom.pose.pose.orientation.z = qz
#         odom.pose.pose.orientation.w = qw

#         odom.twist.twist.linear.x = delta_distance / delta_time
#         odom.twist.twist.angular.z = delta_theta / delta_time

#         # Publish odometry
#         self.odom_publisher.publish(odom)

#         # Publish transform
#         transform = TransformStamped()
#         current_time = Clock().now()  
#         transform.header.stamp = current_time.to_msg()
#         transform.header.frame_id = 'odom'
#         transform.child_frame_id = 'base_link'
#         transform.transform.translation.x = self.x
#         transform.transform.translation.y = self.y

#         # Use the same quaternion for the TF transform
#         transform.transform.rotation.x = qx
#         transform.transform.rotation.y = qy
#         transform.transform.rotation.z = qz
#         transform.transform.rotation.w = qw

#         self.tf_broadcaster.sendTransform(transform)

  
        
#     def shutdown_motors(self):
#         try:
#             self.motor_driver.stop_jogging1()
#             self.motor_driver.stop_jogging2()
#             self.motor_driver.reset_encoder1()
#             self.motor_driver.reset_encoder2()
#             self.motor_driver.disable_driver1()
#             self.motor_driver.disable_driver2()
#             self.get_logger().info("Motors stopped and drivers disabled successfully.")
#         except Exception as e:
#             self.get_logger().error(f"Error during motor shutdown: {e}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = MotorControllerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Node is shutting down.")
#     finally:
#         node.shutdown_motors()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()