#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from moon_servo import MoonServoMotor  # Replace with actual module/class from moon_servo
import time  # Import time module to track execution time


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Initialize the motor with the desired parameters (passing port and baudrate directly)
        self.motor_driver = MoonServoMotor(port='/dev/ttyUSB0', baudrate=115200, base_address=0)  # Use port and baudrate here
        self.motor_driver.connect()
        self.motor_driver.enable_driver1()
        self.motor_driver.enable_driver2()
        self.motor_driver.start_jogging1()
        self.motor_driver.start_jogging2()

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

            self.get_logger().info(f"Received linear velocity from cmd_vel: {linear_velocity}")
            self.get_logger().info(f"Received angular velocity from cmd_vel: {angular_velocity}")

            # Convert cmd_vel to motor commands
            left_motor_speed, right_motor_speed = self.calculate_motor_speeds(linear_velocity, angular_velocity)

            self.get_logger().info(f"Left motor speed: {left_motor_speed}")
            self.get_logger().info(f"Right motor speed: {right_motor_speed}")

            # Measure the time taken to execute set_speed1 (Left motor)
            start_time1 = time.time()  # Record start time for left motor speed
            self.motor_driver.set_speed1(left_motor_speed)
            end_time1 = time.time()  # Record end time for left motor speed
            elapsed_time1 = end_time1 - start_time1  # Calculate elapsed time for left motor speed

            # Measure the time taken to execute set_speed2 (Right motor)
            start_time2 = time.time()  # Record start time for right motor speed
            self.motor_driver.set_speed2(right_motor_speed)
            end_time2 = time.time()  # Record end time for right motor speed
            elapsed_time2 = end_time2 - start_time2  # Calculate elapsed time for right motor speed

            self.get_logger().info(f"Set motor speeds: Left={left_motor_speed}, Right={right_motor_speed}")
            self.get_logger().info(f"Time taken for set_speed1 (Left motor): {elapsed_time1:.6f} seconds")
            self.get_logger().info(f"Time taken for set_speed2 (Right motor): {elapsed_time2:.6f} seconds")
    
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
    
    def cmd_vel_to_rps(linear_velocity, angular_velocity, wheel_radius, wheel_base):
        """
        Convert cmd_vel (linear and angular velocity) to left and right motor RPS.

        Parameters:
            linear_velocity (float): Linear velocity in m/s.
            angular_velocity (float): Angular velocity in rad/s.
            wheel_radius (float): Radius of the wheel in meters.
            wheel_base (float): Distance between the left and right wheels in meters.

        Returns:
            tuple: Left motor RPS, Right motor RPS.
        """
        # Calculate left and right wheel velocities
        v_left = linear_velocity - (angular_velocity * wheel_base / 2)
        v_right = linear_velocity + (angular_velocity * wheel_base / 2)

        # Convert wheel velocities to revolutions per second (RPS)
        rps_left = v_left / (2 * 3.14159 * wheel_radius)
        rps_right = v_right / (2 * 3.14159 * wheel_radius)

        return rps_left, rps_right


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
