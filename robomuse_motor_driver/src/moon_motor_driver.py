#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import logging
from moon_servo import MoonServoMotor 
import time
import os

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s")

class RobomuseMotorNode(Node):
    def __init__(self):
        super().__init__('robomuse_motor_driver')

        # Declare parameters
        self.declare_parameter('robot_port', '/dev/ttyUSB0')
        self.declare_parameter('cmd_vel_timeout', 0.2)  # Timeout in seconds (e.g., 0.2 seconds)

        # Get parameters
        robot_port = self.get_parameter('robot_port').get_parameter_value().string_value
        cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value

        self.robot_cmd_try = 3
        self.get_logger().info(f"Initializing the Robomuse Robot on port: {robot_port}")

        if not os.path.exists(robot_port):
            self.get_logger().error(f"Port {robot_port} not found.")
            self.shutdown_node("Shutting down Robot Node, since port is not available.")  # Shutdown node gracefully
            return

        # Initialize the robot
        try:
            self.robot = MoonServoMotor(port='/dev/ttyUSB0', baudrate=115200, base_address=0)  # Use port and baudrate here
            self.get_logger().info("Robot initialized successfully!")
            #self.get_logger().info(f"Robot Info: {self.robot.get_info()}")

            # Connect to the robot
            self.robot.connect()
            self.robot.enable_driver1()
            self.robot.enable_driver2()
            self.robot.start_jogging1()
            self.robot.start_jogging2()
   
            time.sleep(2)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize the robot: {e}")
            self.shutdown_node("Robot initialization failed.")  # Shutdown node gracefully
            return

        # ROS publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # TF broadcaster
        self.odom_broadcaster = TransformBroadcaster(self)

        # Parameters for robot state
        self.last_time = self.get_clock().now()
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.vel_x, self.vel_y, self.omega = 0.0, 0.0, 0.0

        # Timer to publish odometry at 40 Hz
        self.timer = self.create_timer(1/20, self.publish_odometry) 

    def cmd_vel_callback(self, msg):
        """Callback for /cmd_vel to set robot velocity."""
        # Update last time a command was received
        self.last_time = self.get_clock().now()

        # Convert ROS frame velocities to robot frame velocities
        self.vel_x = msg.linear.y
        self.vel_y = msg.linear.x
        self.omega = msg.angular.z

        self.get_logger().info(f"Published cmd_vel to robot: x:{self.vel_x:.2f}, y:{self.vel_y:.2f}, w:{self.omega:.2f}")
        
        # try:
        #     # Setting Velocity relative to robot base / base_link
        #     for _ in range(self.robot_cmd_try):
        #         self.robot.set_vel_relative(self.vel_x, self.vel_y, self.omega, acc=500, dec=500)
        # except Exception as e:
        #     self.get_logger().error(f"Error setting velocity: {e}")

    def publish_odometry(self):
        """Publishes odometry information."""

        current_time = self.get_clock().now()

        # Get robot pose in its local frame
        encoder1 = self.robot.get_encoder1()
        encoder2 = self.robot.get_encoder2()

        target_wheel_speeds = self.calculate_close_loop_speeds(encoder1, encoder2)



        self.get_logger().info(f"Encoder1 Value:{encoder1}, Encoder2 Value:{encoder2}, ")
        robot_pose = self.get_robot_pose(encoder1, encoder2)

        self.get_logger().info(f"Robot Pose from Hardware API: {robot_pose}")

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"

        # Pose
        
        odom.pose.pose.position.x = float(robot_pose["x"])
        odom.pose.pose.position.y = float(robot_pose["y"])

        
        odom.pose.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0, 0, robot_pose["w"])
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        # Velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = 0.0  # Robot's +x velocity becomes ROS's +y
        odom.twist.twist.linear.y = 0.0  # Robot's +y velocity becomes ROS's +x
        odom.twist.twist.angular.z = 0.0

        # Publish odometry

        # self.get_logger().info(f"Publishing ODOM, Position X:{odom.pose.pose.position.x},Y:{odom.pose.pose.position.y}, Omega:{ros_theta}")
        self.odom_pub.publish(odom)

        # Broadcast transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = float(robot_pose["x"])  # Cast to float
        transform.transform.translation.y = float(robot_pose["y"])         
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        # Send the transform
        self.odom_broadcaster.sendTransform(transform)

    def shutdown_node(self, msg):
        """Shutdown the node gracefully with a custom message."""
        self.get_logger().error(msg)
        self.destroy_node()
        try:
            rclpy.shutdown()  # Safe shutdown attempt
        except RuntimeError:
            pass  # Ignore error if rclpy is not initialized
        exit(1)  # Optionally, exit with a non-zero status to indicate failure

    def get_robot_pose(self, encoder1, encoder2):
        """Calculate robot pose based on encoder values."""
        wheel_radius = 0.05
        wheel_base = 0.5
        
        return {"x":0, "y":0, "w":0}
    
def main(args=None):
    rclpy.init(args=args)
    try:
        robot_node = RobomuseMotorNode()
        rclpy.spin(robot_node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()  # Safe shutdown attempt
        except RuntimeError:
            pass  # Ignore error if rclpy is not initialized

if __name__ == "__main__":
    main()