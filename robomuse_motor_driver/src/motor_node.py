#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from moon_servo import MoonServoMotor  # Replace with the actual module/class from moon_servo
import math


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Motor initialization
        self.motor_driver = MoonServoMotor(port='/dev/ttyUSB0', baudrate=115200, base_address=0)

        # Connect to the motor driver
        self.motor_driver.connect()

        # Enable the motor drivers
        self.motor_driver.enable_driver1()
        self.motor_driver.enable_driver2()

        # Reset motor encoders to ensure accurate motion tracking
        self.motor_driver.reset_encoder1()
        self.motor_driver.reset_encoder2()

        # Set default acceleration and deceleration values for smooth motion control
        self.motor_driver.set_acceleration1(300)
        self.motor_driver.set_deceleration1(300)
        self.motor_driver.set_acceleration2(300)
        self.motor_driver.set_deceleration2(300)

        # Start jogging mode for both motors (continuous motion control)
        self.motor_driver.start_jogging1()
        self.motor_driver.start_jogging2()

        # Store current speeds
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0

        # Robot parameters
        self.wheel_diameter = 0.125  # 125 mm wheel diameter
        self.wheel_base = 0.5  # Distance between the two wheels
        self.encoder_resolution = 65536  # 2^16 for 16-bit encoder resolution


        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ROS publishers
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10  # QoS depth
        )
        self.get_logger().info("MotorControllerNode has been started.")

        # Timer for publishing odometry
        self.create_timer(0.05, self.publish_odometry)  # Publish at 20 Hz

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
        left_motor_speed = linear - (angular * self.wheel_base / 2.0)
        right_motor_speed = linear + (angular * self.wheel_base / 2.0)
        return left_motor_speed, right_motor_speed

    def publish_odometry(self):
        """
        Compute and publish odometry information.
        """
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Get encoder ticks
        left_ticks = self.motor_driver.get_encoder1()
        right_ticks = self.motor_driver.get_encoder2()

        # Log encoder values
        self.get_logger().info(f"Left encoder ticks: {left_ticks}")
        self.get_logger().info(f"Right encoder ticks: {right_ticks}")


        # Convert ticks to distances
        left_distance = (left_ticks / self.encoder_resolution) * math.pi * self.wheel_diameter
        right_distance = (right_ticks / self.encoder_resolution) * math.pi * self.wheel_diameter

        # Compute change in pose
        delta_left = left_distance
        delta_right = right_distance
        delta_distance = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_base

        # Update pose
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)
        self.theta += delta_theta

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Set velocity
        odom.twist.twist.linear.x = delta_distance / delta_time
        odom.twist.twist.angular.z = delta_theta / delta_time

        # Publish odometry
        self.odom_publisher.publish(odom)

        # Publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

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
