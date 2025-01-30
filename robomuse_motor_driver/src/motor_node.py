#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from moon_servo import MoonServoMotor  # Replace with actual module/class from moon_servo
import math


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
        self.motor_driver.set_acceleration1(300)
        self.motor_driver.set_deceleration1(300)
        self.motor_driver.set_acceleration2(300)
        self.motor_driver.set_deceleration2(300)
        self.motor_driver.start_jogging1()
        self.motor_driver.start_jogging2()

        # Robot parameters
        self.wheel_diameter = 0.125  # 125 mm = 0.125 m
        self.wheel_base = 0.5  # Distance between the two wheels
        self.gear_ratio = 20   # 1:20 Gear Ratio
        self.encoder_ppr = 10132  # Computed encoder resolution considering gearbox ratio

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Previous encoder values
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

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
        self.create_timer(0.05, self.publish_odometry)  # 20 Hz

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
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Get encoder ticks
        left_ticks = self.motor_driver.get_encoder1()
        right_ticks = self.motor_driver.get_encoder2()


        # Log encoder values
        self.get_logger().info(f"Left encoder ticks: {left_ticks}")
        self.get_logger().info(f"Right encoder ticks: {right_ticks}")        

        # Compute tick differences
        delta_left_ticks = left_ticks - self.prev_left_ticks
        delta_right_ticks = right_ticks - self.prev_right_ticks

        # Store previous values
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        # Convert ticks to distances
        left_distance = (delta_left_ticks / self.encoder_resolution) * math.pi * self.wheel_diameter
        right_distance = (delta_right_ticks / self.encoder_resolution) * math.pi * self.wheel_diameter

        # Compute change in pose
        delta_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base

        # Update pose
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)
        self.theta += delta_theta

        # Log the differences
        self.get_logger().info(f"Encoder Tick Differences: Left={delta_left_ticks}, Right={delta_right_ticks}")

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
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
        try:
            self.motor_driver.stop_jogging1()
            self.motor_driver.stop_jogging2()
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
