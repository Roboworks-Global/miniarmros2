#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import math

class Nav2Client(Node):
    """
    A simple ROS2 client to send a series of navigation goals to the Nav2 action server.
    """

    def __init__(self):
        super().__init__('nav2_client')
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        self.get_logger().info('Nav2 client node has been initialized.')

        # List of goals to send, each as a tuple (x, y, yaw_degrees)
        self.goals = [
            (0.4, 0.0, 0.0),
            (0.4, 0.0, 60.0),
            (0.8, 0.0, 0.0)
        ]
        self.goal_index = 0

    def send_next_goal(self):
        """
        Sends the next goal in the list if available.
        """
        if self.goal_index < len(self.goals):
            x, y, yaw_degrees = self.goals[self.goal_index]
            self.get_logger().info("Waiting for action server...")
            if not self._action_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("Action server not available. Is Nav2 running?")
                return

            self.get_logger().info("Action server is available. Sending goal...")

            goal_msg = NavigateToPose.Goal()

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            pose_stamped.pose.position.z = 0.0

            # Convert yaw from degrees to quaternion
            yaw_rad = math.radians(yaw_degrees)
            pose_stamped.pose.orientation.z = math.sin(yaw_rad / 2.0)
            pose_stamped.pose.orientation.w = math.cos(yaw_rad / 2.0)

            goal_msg.pose = pose_stamped

            self.get_logger().info(f"Sending goal {self.goal_index + 1} to (x={x}, y={y}, yaw={yaw_degrees} degrees)")
            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info("All goals have been sent successfully. Shutting down node.")
            rclpy.shutdown()

    def goal_response_callback(self, future):
        """
        Callback to handle the goal response from the action server.
        Checks if the goal was accepted or rejected.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the server')
            return

        self.get_logger().info(f'Goal {self.goal_index + 1} accepted. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Callback to process feedback from the action server.
        This provides real-time updates on the robot's progress.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Received feedback: Distance remaining = {feedback.distance_remaining:.2f} meters'
        )

    def get_result_callback(self, future):
        """
        Callback to handle the final result of the action.
        """
        status = future.result().status
        if status == 4: # GoalStatus.SUCCEEDED
            self.get_logger().info(f'Goal {self.goal_index + 1} succeeded!')
            self.goal_index += 1
            self.send_next_goal()
        else:
            self.get_logger().error(f'Goal {self.goal_index + 1} failed with status: {status}')
            self.get_logger().info("Aborting remaining goals.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    nav2_client = Nav2Client()
    nav2_client.send_next_goal()

    # Use a loop to keep the node spinning while waiting for results
    try:
        rclpy.spin(nav2_client)
    except KeyboardInterrupt:
        pass
    finally:
        nav2_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
