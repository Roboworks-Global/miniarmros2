#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray
from control_msgs.msg import GripperCommand
import time

WAYPOINTS = [
    {'x': 1.0, 'y': 1.0},
    {'x': 2.0, 'y': 1.0},
    {'x': 2.0, 'y': 2.0},
    {'x': 1.0, 'y': 2.0}
]

BIN_LOCATION = {'x': 0.0, 'y': 0.0}
GRIPPER_OPEN = 0.08
GRIPPER_CLOSED = 0.02
APPROACH_DISTANCE = 0.3

class DiceCollector(Node):
    def __init__(self):
        super().__init__('dice_collector')
        
        # Navigation client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Vision subscriber
        self.detected_dice = None
        self.detection_sub = self.create_subscription(
            Detection3DArray,
            'detections',
            self.detection_callback,
            10
        )
        
        # Gripper publisher
        self.gripper_pub = self.create_publisher(
            GripperCommand,
            'gripper_command',
            10
        )

    def detection_callback(self, msg):
        for detection in msg.detections:
            if detection.id == "red":
                self.detected_dice = detection
                self.get_logger().info('Red dice detected!')

    def control_gripper(self, open_width):
        msg = GripperCommand()
        msg.position = open_width
        msg.max_effort = 50.0
        self.gripper_pub.publish(msg)
        time.sleep(1.0)

    def navigate_to(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def pick_dice(self):
        if not self.detected_dice:
            return False
            
        # Navigate to approach pose
        target = self.detected_dice.bbox.center.position
        self.navigate_to(target.x - APPROACH_DISTANCE, target.y)
        
        # Pick sequence (placeholder for MoveIt integration)
        self.control_gripper(GRIPPER_OPEN)
        # Add MoveIt arm movement here
        
        # Close gripper
        self.control_gripper(GRIPPER_CLOSED)
        
        return True

    def return_to_bin(self):
        # Navigate to bin
        self.navigate_to(BIN_LOCATION['x'], BIN_LOCATION['y'])
        
        # Drop sequence
        self.control_gripper(GRIPPER_OPEN)
        self.detected_dice = None

def main():
    rclpy.init()
    collector = DiceCollector()
    
    try:
        # Wait for navigation action server
        collector.get_logger().info('Waiting for navigation server...')
        collector.nav_client.wait_for_server()
        while rclpy.ok():
            # Exploration loop
            for waypoint in WAYPOINTS:
                # Check if dice detected
                rclpy.spin_once(collector, timeout_sec=0)  # Process any callbacks
                if collector.detected_dice:
                    collector.get_logger().info('Breaking exploration to pick dice')
                    break
                    
                # Move to next waypoint
                collector.get_logger().info(f'Moving to waypoint: {waypoint}')
                collector.navigate_to(waypoint['x'], waypoint['y'])
            
            # If dice detected, handle it
            if collector.detected_dice:
                if collector.pick_dice():
                    collector.return_to_bin()
                collector.detected_dice = None  # Reset detection
    
    except KeyboardInterrupt:
        pass
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()