import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        
        # Declare parameters with expanded angles
        self.declare_parameter('filter_angles', [-2.7, -2.3, -0.8, -0.4, 0.4, 0.8, 2.3, 2.7])  # Expanded range
        
        # Subscriber to the raw Lidar scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.lidar_callback,
            10)
        
        # Publisher for the filtered scan
        self.publisher = self.create_publisher(LaserScan, '/scan_filtered', 10)
        
        self.get_logger().info("Lidar filter node started")

    def lidar_callback(self, msg: LaserScan):
        # Retrieve filter angles and reconstruct as pairs
        raw_values = self.get_parameter('filter_angles').get_parameter_value().double_array_value
        filter_ranges = list(zip(raw_values[::2], raw_values[1::2]))  #  Reconstruct pairs

        filtered_msg = msg
        angle_increment = msg.angle_increment
        num_points = len(msg.ranges)

        for start_angle, end_angle in filter_ranges:
            start_index = int((start_angle - msg.angle_min) / angle_increment)
            end_index = int((end_angle - msg.angle_min) / angle_increment)
            
            # Ensure index bounds are valid
            for i in range(max(0, start_index), min(num_points, end_index)):
                filtered_msg.ranges[i] = float('inf')  # Set to 'inf' to ignore points
        
        # Publish the filtered scan
        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
