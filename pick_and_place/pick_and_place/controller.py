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
        self.detected_dices = []
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

        # State management
        self.current_waypoint = 0
        self.is_busy = False
        self.current_nav_goal = None  # Store current navigation goal handle
        
        # Create timer for exploration
        self.timer = self.create_timer(1.0, self.timer_callback)

    def detection_callback(self, msg):
        if self.is_busy:
            return
            
        for detection in msg.detections:
            self.detected_dices.append(detection)
        self.get_logger().info('Red dice detected!')
        # Cancel current navigation if exploring
        if self.current_nav_goal is not None:
            self.current_nav_goal.cancel_goal_async()
            self.current_nav_goal = None

    def control_gripper(self, open_width):
        msg = GripperCommand()
        msg.position = open_width
        msg.max_effort = 50.0
        self.gripper_pub.publish(msg)
        time.sleep(1.0)

    def create_nav_goal(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def navigate_sync(self, x, y):
        """Synchronized navigation - waits for completion"""
        goal = self.create_nav_goal(x, y)
        
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def navigation_response_callback(self, future):
        """Callback for handling navigation responses during exploration"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Exploration goal rejected')
            self.current_nav_goal = None
            return

        self.current_nav_goal = goal_handle
        
        # Get the result future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Callback for handling navigation completion during exploration"""
        self.current_nav_goal = None
        # Update waypoint only if navigation completed successfully
        # (not cancelled due to dice detection)
        if not self.detected_dice:
            self.current_waypoint = (self.current_waypoint + 1) % len(WAYPOINTS)

    def pick_dice(self):
        if not self.detected_dice:
            return False
            
        # Navigate to approach pose (synchronized)
        target = self.detected_dice.bbox.center.position
        if not self.navigate_sync(target.x - APPROACH_DISTANCE, target.y):
            return False
        
        # Pick sequence
        self.control_gripper(GRIPPER_OPEN)
        # Add MoveIt arm movement here
        
        # Close gripper
        self.control_gripper(GRIPPER_CLOSED)
        return True

    def return_to_bin(self):
        # Navigate to bin (synchronized)
        if not self.navigate_sync(BIN_LOCATION['x'], BIN_LOCATION['y']):
            return False
        
        # Drop sequence
        self.control_gripper(GRIPPER_OPEN)
        self.detected_dice = None
        return True

    def timer_callback(self):
        # Skip if busy with another operation
        if self.is_busy or self.current_nav_goal is not None:
            return

        try:
            self.is_busy = True
            
            # Check if dice detected
            if len(self.detected_dices) > 0:
                self.get_logger().info('Attempting to pick dice')
                self.detected_dice = self.detected_dices.pop()
                if self.pick_dice():
                    self.return_to_bin()
                self.detected_dice = None
            else:
                # Start async navigation to next waypoint
                waypoint = WAYPOINTS[self.current_waypoint]
                self.get_logger().info(f'Moving to waypoint: {waypoint}')
                
                # Send goal asynchronously
                goal = self.create_nav_goal(waypoint['x'], waypoint['y'])
                self.nav_client.wait_for_server()
                future = self.nav_client.send_goal_async(goal)
                future.add_done_callback(self.navigation_response_callback)
                
        finally:
            self.is_busy = False

def main():
    rclpy.init()
    collector = DiceCollector()
    
    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        pass
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()