import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import time

class NavigateRobot(Node):
    def __init__(self):
        super().__init__('navigate_robot')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_positions = [
            {'name': 'home', 'x': 2.2807858546988813, 'y': 0.6472089629028978, 'z': -0.3826575954518254, 'w': 0.9238902340879176},
            {'name': 'Sofa', 'x': 1.250403441847467, 'y': -1.1690667364960936, 'z': -0.9774214104091838, 'w': 0.2112992817633836},
            {'name': 'TV', 'x': 3.3154408427238966, 'y': -0.4770062129404991, 'z': -0.9995838160141977, 'w': 0.028847786093470083},
            {'name': 'Door', 'x': 0.08727737310636037, 'y': -0.2798405278232123, 'z': -0.014814243485838532, 'w': 0.9998902630738746}
        ]
        self.current_goal_index = 0  # Start from the first goal
        self.returning_home = False  # Flag to track return to home
        self.send_next_goal()

    def send_next_goal(self):
        """Send the next goal in the list, then return home after the last goal."""
        if self.current_goal_index >= len(self.goal_positions):
            self.get_logger().info("✅ Navigation complete! Stopping the robot.")
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

        self.get_logger().info("⏳ Waiting for 5 seconds before proceeding to the next goal...")
        time.sleep(5)  # Halt for 5 seconds

        # If the last goal was reached and we haven't returned home yet, add home as the final goal
        if self.current_goal_index == len(self.goal_positions) - 1 and not self.returning_home:
            self.get_logger().info("🔄 Returning to Home...")
            self.goal_positions.append(self.goal_positions[0])  # Append "home" as the final goal
            self.returning_home = True  # Set flag to prevent duplicate additions

        self.current_goal_index += 1  # Move to the next goal
        self.send_next_goal()  # Send the next goal


def main():
    rclpy.init()
    navigator = NavigateRobot()
    rclpy.spin(navigator)  # Keep the node running until all goals are reached

if __name__ == '__main__':
    main()





##############################################################
#############################continuous in loop#######################
###################################################################

# import rclpy
# import time
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from geometry_msgs.msg import PoseStamped
# from nav2_msgs.action import NavigateToPose

# class NavigateRobot(Node):
#     def __init__(self):
#         super().__init__('navigate_robot')
#         self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
#         self.goal_positions = [
#             {'name': 'home', 'x': 0.4058, 'y': -0.2415, 'z': 0.0027, 'w': 0.9999},
#             {'name': 'Sofa', 'x': 2.6413, 'y': 0.7004, 'z': -0.6940, 'w': 0.7199},
#             {'name': 'TV', 'x': 4.7903, 'y': -0.2082, 'z': 0.9990, 'w': 0.0430},
#             {'name': 'Door', 'x': 1.4313, 'y': -2.2513, 'z': 0.6841, 'w': 0.7293}
#         ]
#         self.current_goal_index = 0  # Start from the first goal
#         self.send_next_goal()

#     def send_next_goal(self):
#         """Send the next goal and loop indefinitely."""
#         if self.current_goal_index >= len(self.goal_positions):
#             self.get_logger().info("🔁 Completed one loop. Restarting from the first goal...")
#             self.current_goal_index = 0  # Restart from the first goal

#         position = self.goal_positions[self.current_goal_index]
#         self.get_logger().info(f"🚀 Sending goal to {position['name']}: ({position['x']}, {position['y']})")

#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = 'map'
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
#         goal_msg.pose.pose.position.x = position['x']
#         goal_msg.pose.pose.position.y = position['y']
#         goal_msg.pose.pose.orientation.z = position['z']
#         goal_msg.pose.pose.orientation.w = position['w']

#         if not self.client.wait_for_server(timeout_sec=10.0):
#             self.get_logger().error("❌ Action server not available! Exiting...")
#             rclpy.shutdown()
#             return

#         send_goal_future = self.client.send_goal_async(goal_msg)
#         send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         """Handles the response from the action server"""
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info("⚠️ Goal rejected!")
#             return
#         self.get_logger().info("✅ Goal accepted, waiting for result...")

#         get_result_future = goal_handle.get_result_async()
#         get_result_future.add_done_callback(self.result_callback)

#     def result_callback(self, future):
#         """Handles the result when the goal is reached or failed"""
#         result = future.result()
#         if result:
#             self.get_logger().info("🎯 Goal successfully reached!")
#         else:
#             self.get_logger().info("❌ Goal failed!")

#         # Wait for 5 seconds before moving to the next goal
#         self.get_logger().info("⏳ Waiting for 5 seconds at the goal...")
#         time.sleep(5)

#         self.current_goal_index += 1  # Move to the next goal
#         self.send_next_goal()  # Send the next goal

# def main():
#     rclpy.init()
#     navigator = NavigateRobot()
#     rclpy.spin(navigator)  # Keep the node running in a loop

# if __name__ == '__main__':
#     main()



#########################move in one cycle#############
#####################################################################
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from geometry_msgs.msg import PoseStamped
# from nav2_msgs.action import NavigateToPose
# from rclpy.timer import Timer
# import time

# class NavigateRobot(Node):
#     def __init__(self):
#         super().__init__('navigate_robot')
#         self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
#         self.goal_positions = [
#             {'name': 'home', 'x': 0.4058098261317316, 'y': -0.24153020727258429, 'z': 0.00278071876100996, 'w': 0.9999961337941123},
#             {'name': 'Sofa', 'x': 2.6413651604488093, 'y': 0.7004190268441755, 'z': -0.6940663812029765, 'w': 0.7199110073361877},
#             {'name': 'TV', 'x': 4.790321201397294, 'y': -0.20827729058528402, 'z': 0.9990731248784613, 'w': 0.04304522209939696},
#             {'name': 'Door', 'x': 1.4313656908337922, 'y': -2.251369138635757, 'z': 0.6841051647081836, 'w': 0.7293833858949551}
#         ]
#         self.current_goal_index = 0  # Start from the first goal
#         self.send_next_goal()

#     def send_next_goal(self):
#         """Send the next goal in the list, then stop after the last goal."""
#         if self.current_goal_index >= len(self.goal_positions):
#             self.get_logger().info("✅ All goals reached! Stopping navigation.")
#             rclpy.shutdown()
#             return

#         position = self.goal_positions[self.current_goal_index]
#         self.get_logger().info(f"🚀 Sending goal to {position['name']}: ({position['x']}, {position['y']})")

#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = 'map'
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
#         goal_msg.pose.pose.position.x = position['x']
#         goal_msg.pose.pose.position.y = position['y']
#         goal_msg.pose.pose.orientation.z = position['z']
#         goal_msg.pose.pose.orientation.w = position['w']

#         if not self.client.wait_for_server(timeout_sec=10.0):
#             self.get_logger().error("❌ Action server not available! Exiting...")
#             rclpy.shutdown()
#             return

#         send_goal_future = self.client.send_goal_async(goal_msg)
#         send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         """Handles the response from the action server"""
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info("⚠️ Goal rejected!")
#             return
#         self.get_logger().info("✅ Goal accepted, waiting for result...")

#         get_result_future = goal_handle.get_result_async()
#         get_result_future.add_done_callback(self.result_callback)

#     def result_callback(self, future):
#         """Handles the result when the goal is reached or failed"""
#         result = future.result()
#         if result:
#             self.get_logger().info("🎯 Goal successfully reached!")
#         else:
#             self.get_logger().info("❌ Goal failed!")
        
#         self.get_logger().info("⏳ Waiting for 5 seconds before proceeding to the next goal...")
#         time.sleep(5)  # Halt for 5 seconds
        
#         self.current_goal_index += 1  # Move to the next goal
#         self.send_next_goal()  # Send the next goal


# def main():
#     rclpy.init()
#     navigator = NavigateRobot()
#     rclpy.spin(navigator)  # Keep the node running until all goals are reached

# if __name__ == '__main__':
#     main()
