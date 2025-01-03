#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from moon_servo import MotorDriver  # Replace with actual module/class from moon_servo

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Initialize the motor driver
        self.motor_driver = MotorDriver()  # Replace with actual initialization code

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10  # QoS
        )
        self.subscription  # Prevent unused variable warning

        self.get_logger().info("MotorControllerNode has been started.")

    def cmd_vel_callback(self, msg):
        """
        Callback function to handle incoming cmd_vel messages and send commands to motor driver.
        """
        try:
            linear_velocity = msg.linear.x  # Forward/backward velocity
            angular_velocity = msg.angular.z  # Rotational velocity

            # Convert cmd_vel to motor commands
            left_motor_speed, right_motor_speed = self.calculate_motor_speeds(linear_velocity, angular_velocity)

            # Send commands to the motor driver
            self.motor_driver.set_motor_speed(left_motor_speed, right_motor_speed)

            self.get_logger().info(f"Set motor speeds: Left={left_motor_speed}, Right={right_motor_speed}")
        except Exception as e:
            self.get_logger().error(f"Error in cmd_vel_callback: {e}")

    def calculate_motor_speeds(self, linear, angular):
        """
        Convert linear and angular velocity to left and right motor speeds.
        Adjust the formula based on your motor setup and robot geometry.
        """
        wheel_base = 0.5  # Distance between wheels in meters (adjust as per your robot)
        
        # Differential drive formula
        left_motor_speed = linear - (angular * wheel_base / 2.0)
        right_motor_speed = linear + (angular * wheel_base / 2.0)
        
        return left_motor_speed, right_motor_speed

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node is shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
