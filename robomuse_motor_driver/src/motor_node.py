#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from moon_servo import MoonServoMotor  # Replace with actual module/class from moon_servo


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Motor initialization
        self.motor_driver = MoonServoMotor(port='/dev/ttyUSB0', baudrate=115200, base_address=0)
        self.motor_driver.connect()
        self.motor_driver.enable_driver1()
        self.motor_driver.enable_driver2()
        self.motor_driver.reset_encoder1()
        self.motor_driver.reset_encoder2()
        self.motor_driver.start_jogging1()
        self.motor_driver.start_jogging2()

        # Store current speeds
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10  # QoS depth
        )
        self.get_logger().info("MotorControllerNode has been started.")

    def cmd_vel_callback(self, msg):
        """
        Handle incoming cmd_vel messages and send commands to the motor driver.
        """
        try:
            linear_velocity = msg.linear.x
            angular_velocity = msg.angular.z

            # Log received velocities
            self.get_logger().info(f"Received linear velocity from cmd_vel: {linear_velocity}")
            self.get_logger().info(f"Received angular velocity from cmd_vel: {angular_velocity}")

            # Convert to motor speeds
            left_motor_speed, right_motor_speed = self.calculate_motor_speeds(linear_velocity, angular_velocity)

            # Determine acceleration or deceleration for the left motor
            if left_motor_speed > self.current_left_speed:
                self.motor_driver.set_acceleration1(300)  # Example acceleration value
            else: 
                self.motor_driver.set_deceleration1(300)  # Example deceleration value

            # Determine acceleration or deceleration for the right motor
            if right_motor_speed > self.current_right_speed:
                self.motor_driver.set_acceleration2(300)  # Example acceleration value
            else:
                self.motor_driver.set_deceleration2(300)  # Example deceleration value

            # Send motor speed commands
            self.motor_driver.set_speed1(left_motor_speed)
            self.motor_driver.set_speed2(right_motor_speed)

            # Update current speeds
            self.current_left_speed = left_motor_speed
            self.current_right_speed = right_motor_speed

            self.get_logger().info(f"Set motor speeds: Left={left_motor_speed}, Right={right_motor_speed}")
        except Exception as e:
            self.get_logger().error(f"Error in cmd_vel_callback: {e}")

    def calculate_motor_speeds(self, linear, angular):
        """
        Convert linear and angular velocity to left and right motor speeds.
        """
        wheel_base = 0.5  # Adjust as per your robot's geometry
        left_motor_speed = linear - (angular * wheel_base / 2.0)
        right_motor_speed = linear + (angular * wheel_base / 2.0)
        return left_motor_speed, right_motor_speed

    def shutdown_motors(self):
        """
        Gracefully stop the motors and disable the drivers.
        """
        try:
            self.motor_driver.stop_jogging1()
            self.motor_driver.stop_jogging2()
            self.motor_driver.disable_driver1()
            self.motor_driver.disable_driver2()
            self.get_logger().info("Motors stopped and drivers disabled successfully.")
        except Exception as e:
            self.get_logger().error(f"Error during motor shutdown: {e}")

    def __del__(self):
        """
        Destructor to ensure motors are stopped when the node is destroyed.
        """
        self.shutdown_motors()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node is shutting down.")
    finally:
        node.shutdown_motors()  # Ensure motors are stopped
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

