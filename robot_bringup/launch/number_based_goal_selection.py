import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class NavigateRobot(Node):
    def __init__(self):
        super().__init__('navigate_robot')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.amcl_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10
        )
        self.initialpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.current_x = 0.0
        self.current_y = 0.0
        self.active_goal = None
        self.initial_distance = None
        
        self.goal_positions = {
            0: {'name': 'Home', 'x': 0.4058098261317316, 'y': -0.24153020727258429, 'z': 0.00278071876100996, 'w': 0.9999961337941123},
            1: {'name': 'Sofa', 'x': 2.6413651604488093, 'y': 0.7004190268441755, 'z': -0.6940663812029765, 'w': 0.7199110073361877},
            2: {'name': 'TV', 'x': 4.790321201397294, 'y': -0.20827729058528402, 'z': 0.9990731248784613, 'w': 0.04304522209939696},
            3: {'name': 'Door', 'x': 1.4313656908337922, 'y': -2.251369138635757, 'z': 0.6841051647081836, 'w': 0.7293833858949551}
        }
        
        self.set_initial_pose()
    
    def set_initial_pose(self):
        """Publishes the initial pose (Home) to help localize the robot."""
        home_pose = self.goal_positions[0]  # Home location
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header = Header()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.pose.pose.position.x = home_pose['x']
        initial_pose_msg.pose.pose.position.y = home_pose['y']
        initial_pose_msg.pose.pose.orientation.z = home_pose['z']
        initial_pose_msg.pose.pose.orientation.w = home_pose['w']
        initial_pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06
        ]
        
        self.initialpose_publisher.publish(initial_pose_msg)
        self.get_logger().info("🏠 Initial pose set to Home")
    
    def amcl_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.print_distance_remaining()
    
    def print_distance_remaining(self):
        if self.active_goal:
            goal_x = self.active_goal['x']
            goal_y = self.active_goal['y']
            current_distance = ((goal_x - self.current_x) ** 2 + (goal_y - self.current_y) ** 2) ** 0.5
            
            if self.initial_distance is None:
                self.initial_distance = current_distance

            if self.initial_distance > 0:
                percentage_remaining = (current_distance / self.initial_distance) * 100
            else:
                percentage_remaining = 0

            self.get_logger().info(f"📏 Distance remaining: {current_distance:.2f} meters")
            self.get_logger().info(f"📊 Task completion: {100 - percentage_remaining:.1f}%")
    
    def send_goal(self, goal_number):
        if goal_number not in self.goal_positions:
            self.get_logger().error(f"❌ Invalid choice: {goal_number}. Choose from {list(self.goal_positions.keys())}")
            return
        
        self.active_goal = self.goal_positions[goal_number]
        self.initial_distance = None
        position = self.active_goal
        self.get_logger().info(f"🚀 Sending goal to {position['name']}: ({position['x']}, {position['y']})")
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = position['x']
        goal_msg.pose.pose.position.y = position['y']
        goal_msg.pose.pose.orientation.z = position['z']
        goal_msg.pose.pose.orientation.w = position['w']
        
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ Action server not available! Exiting...")
            return
        
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("⚠️ Goal rejected!")
            return
        
        self.get_logger().info("✅ Goal accepted, monitoring progress...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info("🎯 Goal successfully reached!")
        else:
            self.get_logger().info("❌ Goal failed!")
        
        self.active_goal = None


def main():
    rclpy.init()
    navigator = NavigateRobot()

    while rclpy.ok():
        print("\n📍 Choose a Goal:")
        print("0️⃣ Home")
        print("1️⃣ Sofa")
        print("2️⃣ TV")
        print("3️⃣ Door")
        print("4️⃣ Exit")

        try:
            goal_number = int(input("\nEnter goal number (0/1/2/3/4): ").strip())
            if goal_number == 4:
                print("🚪 Exiting...")
                break
            elif goal_number not in [0, 1, 2, 3]:
                print("❌ Invalid input! Please enter a number (0, 1, 2, 3, or 4).")
                continue
        except ValueError:
            print("❌ Invalid input! Please enter a number (0, 1, 2, 3, or 4).")
            continue

        navigator.send_goal(goal_number)
        while navigator.active_goal:
            rclpy.spin_once(navigator, timeout_sec=0.5)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
