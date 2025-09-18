#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, ColorRGBA
import time
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener

class DiceCollector2(Node):
    """
    A ROS2 client that sends navigation goals to the Nav2 action server
    based on messages received on a topic.
    """

    def __init__(self):
        super().__init__('dice_collector2')
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        self.get_logger().info('Nav2 client node has been initialized.')

        # # Subscriber to listen for new navigation goals
        # self._goal_subscriber = self.create_subscription(
        #     PoseStamped,
        #     '/goal_topic',
        #     self.goal_topic_callback,
        #     10
        # )
        self.get_logger().info('Subscribed to /goal_topic to receive navigation goals.')

        self._send_goal_future = None
        self._get_result_future = None


        self.detection_sub = self.create_subscription(
            Detection3DArray,
            'detections',
            self.goal_topic_callback,
            10
        )

        # State management
        self.current_waypoint = 0
        self.is_busy = False
        self.current_nav_goal = None
        self.detected_dice = None

        
    def goal_topic_callback(self, msg: Detection3DArray):
        """
        Callback to handle incoming PoseStamped messages and send them
        as navigation goals.
        """
        # self.get_logger().info(
        #     f"Received new goal from topic: x={msg.pose.position.x}, y={msg.pose.position.y}"
        # )
        if self.detected_dice is None and msg.detections is not None:
            for detection in msg.detections:
                try:
                    self.detected_dice = detection.bbox.center
                    # self.detected_dice = PoseStamped()
                    # self.detected_dice.header = msg.header
                    # self.detected_dice.pose = detection.bbox.center
                    self.get_logger().info('Dice detected!')
                    
                except Exception as e:
                    self.get_logger().error(f'Failed to transform detection: {e}')
                    continue

            self.get_logger().info("Waiting for action server...")
            if not self._action_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("Action server not available. Is Nav2 running?")
                return

            self.get_logger().info("Action server is available. Sending goal...")

            goal_msg = NavigateToPose.Goal()

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = float(self.detected_dice.position.x - 0.5)
            pose_stamped.pose.position.y = float(self.detected_dice.position.y)
            pose_stamped.pose.position.z = 0.0

            # Convert yaw from degrees to quaternion
            # yaw_rad = math.radians(yaw_degrees)
            # pose_stamped.pose.orientation.z = math.sin(yaw_rad / 2.0)
            # pose_stamped.pose.orientation.w = math.cos(yaw_rad / 2.0)

            goal_msg.pose = pose_stamped

            # self.get_logger().info(f"Sending goal {self.goal_index + 1} to (x={x}, y={y}, yaw={yaw_degrees} degrees)")

            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback to handle the goal response from the action server.
        Checks if the goal was accepted or rejected.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the server')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
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
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error(f'Goal failed with status: {status}')

def main():
    rclpy.init()
    nav2_client = DiceCollector2()

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
