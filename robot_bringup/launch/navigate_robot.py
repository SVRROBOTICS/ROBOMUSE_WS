import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class NavigateRobot(Node):
    def __init__(self):
        super().__init__('navigate_robot')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_positions = [
            {'name': 'home', 'x': 1.9729116272811456, 'y': 1.5482711157234594, 'z': -0.22630269918781945, 'w': 0.9740570251994014},
            {'name': 'sofa', 'x': 1.9729116272811456, 'y': 1.5482711157234594, 'z': -0.22630269918781945, 'w': 0.9740570251994014},
            {'name': 'tv', 'x': 4.5995795293666655, 'y': -0.4191154852254095, 'z': -0.7615429442280517, 'w': 0.648114452929782},
            {'name': 'door', 'x': 2.3200930056008406, 'y': -1.7127323542781154, 'z': 0.9998315424127682, 'w': 0.9998315424127682}
        ]
        self.current_goal_index = 0  # Start from the first goal
        self.send_next_goal()

    def send_next_goal(self):
        """Send the next goal in the list, then stop after the last goal."""
        if self.current_goal_index >= len(self.goal_positions):
            self.get_logger().info("✅ All goals reached! Stopping navigation.")
            rclpy.shutdown()
            return

        position = self.goal_positions[self.current_goal_index]
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
            rclpy.shutdown()
            return

        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handles the response from the action server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("⚠️ Goal rejected!")
            return
        self.get_logger().info("✅ Goal accepted, waiting for result...")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handles the result when the goal is reached or failed"""
        result = future.result()
        if result:
            self.get_logger().info("🎯 Goal successfully reached!")
        else:
            self.get_logger().info("❌ Goal failed!")

        self.current_goal_index += 1  # Move to the next goal
        self.send_next_goal()  # Send the next goal

def main():
    rclpy.init()
    navigator = NavigateRobot()
    rclpy.spin(navigator)  # Keep the node running until all goals are reached

if __name__ == '__main__':
    main()











# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from geometry_msgs.msg import PoseStamped
# from nav2_msgs.action import NavigateToPose

# class NavigateRobot(Node):
#     def __init__(self):
#         super().__init__('navigate_robot')
#         self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
#         self.goal_positions = [
#             {'name': 'sofa', 'x': 1.9729116272811456, 'y': 1.5482711157234594, 'z': -0.22630269918781945, 'w': 0.9740570251994014},
#             {'name': 'tv', 'x': 4.5995795293666655, 'y': -0.4191154852254095, 'z': -0.7615429442280517, 'w': 0.648114452929782},
#             {'name': 'door', 'x': 2.3200930056008406, 'y': -1.7127323542781154, 'z': 0.9998315424127682, 'w': 0.9998315424127682}
#         ]
    
#     def send_goal(self, position):
#         """Sends a navigation goal to the robot"""
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = 'map'
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
#         goal_msg.pose.pose.position.x = position['x']
#         goal_msg.pose.pose.position.y = position['y']
#         goal_msg.pose.pose.orientation.z = position['z']
#         goal_msg.pose.pose.orientation.w = position['w']

#         self.get_logger().info(f"Sending goal to {position['name']}: ({position['x']}, {position['y']})")

#         # Ensure action server is available
#         if not self.client.wait_for_server(timeout_sec=10.0):
#             self.get_logger().error("Action server not available! Exiting...")
#             return False
        
#         # Send goal and wait for response
#         self.send_goal_future = self.client.send_goal_async(goal_msg)
#         self.send_goal_future.add_done_callback(self.goal_response_callback)

#         return True  # Goal sent successfully

#     def goal_response_callback(self, future):
#         """Handles the response from the action server"""
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected by the server')
#             return
#         self.get_logger().info('Goal accepted, waiting for result...')

#         self.get_result_future = goal_handle.get_result_async()
#         self.get_result_future.add_done_callback(self.result_callback)

#     def result_callback(self, future):
#         """Handles the result when the goal is reached or failed"""
#         result = future.result()
#         if result:
#             self.get_logger().info('Goal successfully reached!')
#         else:
#             self.get_logger().info('Goal failed!')

# def main():
#     rclpy.init()
#     navigator = NavigateRobot()

#     for position in navigator.goal_positions:
#         success = navigator.send_goal(position)
#         if success:
#             rclpy.spin(navigator)  # Wait until the goal is reached before sending the next one

#     navigator.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
