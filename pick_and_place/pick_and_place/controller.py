#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from vision_msgs.msg import Detection3DArray
from std_msgs.msg import Bool
import time
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener

WAYPOINTS = [
    {'x': 1.0, 'y': 1.0},
    {'x': 2.0, 'y': 1.0},
    {'x': 2.0, 'y': 2.0},
    {'x': 1.0, 'y': 2.0}
]

BIN_LOCATION = {'x': 0.0, 'y': 0.0}
APPROACH_DISTANCE = 0.3

class DiceCollector(Node):
    def __init__(self):
        super().__init__('dice_collector')
        
        # Navigation client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Vision subscriber
        self.detected_dice = None
        self.detection_sub = self.create_subscription(
            Detection3DArray,
            'detections',
            self.detection_callback,
            10
        )
        
        # Arm control publishers
        self.arm_target_pub = self.create_publisher(
            Point,
            'arm_target_point',
            10
        )
        self.gripper_pub = self.create_publisher(
            Bool,
            'gripper_command_close',
            10
        )

        # State management
        self.current_waypoint = 0
        self.is_busy = False
        self.current_nav_goal = None
        
        # Create timer for exploration
        self.timer = self.create_timer(1.0, self.timer_callback)

    def control_gripper(self, close_gripper):
        """Control gripper open/close"""
        msg = Bool()
        msg.data = close_gripper  # True to close, False to open
        self.gripper_pub.publish(msg)
        time.sleep(1.0)  # Wait for gripper to complete movement
        return True
        
    def move_arm_to_target(self, x, y, z):
        """Move arm to target position"""
        target_point = Point()
        target_point.x = x
        target_point.y = y
        target_point.z = z
        
        self.arm_target_pub.publish(target_point)
        time.sleep(2.0)  # Wait for arm to complete movement
        return True
    
    def move_to_home_position(self):
        """Move the arm to its home position"""
        return self.move_arm_to_target(0.3, 0.0, 0.4)  # Adjust these values based on your robot
    
    def create_pick_pose(self, target_pose):
        """Convert target pose to arm coordinates"""
        # Note: You'll need to transform the target pose from map frame 
        # to arm base frame before using these coordinates
        try:
            transform = self.tf_buffer.lookup_transform(
                'arm_base_link',  # Adjust this arm's base frame
                target_pose.header.frame_id,
                target_pose.header.stamp
            )
            transformed_pose = tf2_geometry_msgs.do_transform_pose(target_pose, transform)
            return transformed_pose.pose
            
        except Exception as e:
            self.get_logger().error(f'Failed to transform pick pose: {e}')
            return None
    
    def detection_callback(self, msg):
        # Ignore detections if we're busy or already have a target
        if self.is_busy or self.detected_dice is not None:
            return

        try:
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f'Failed to lookup transform: {e}')
            return

        # Take the first valid detection
        for detection in msg.detections:
            try:
                detection_map = tf2_geometry_msgs.do_transform_pose(detection.bbox.center, transform)
                self.detected_dice = detection_map
                self.get_logger().info('Dice detected!')
                
                # Cancel current navigation if exploring
                if self.current_nav_goal is not None:
                    self.current_nav_goal.cancel_goal_async()
                    self.current_nav_goal = None
                break
                
            except Exception as e:
                self.get_logger().error(f'Failed to transform detection: {e}')
                continue

    def create_nav_goal(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        return NavigateToPose.Goal(pose=pose)

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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Exploration goal rejected')
            self.current_nav_goal = None
            return

        self.current_nav_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        self.current_nav_goal = None
        if not self.detected_dice:
            self.current_waypoint = (self.current_waypoint + 1) % len(WAYPOINTS)

    def pick_dice(self):
        """Pick up the detected dice"""
        if not self.detected_dice:
            return False
            
        try:
            # Navigate to approach pose
            target = self.detected_dice.position
            if not self.navigate_sync(target.x - APPROACH_DISTANCE, target.y):
                return False
            
            pick_pose = self.create_pick_pose(self.detected_dice)
            if not pick_pose:
                return False
            
            # Pre-grasp
            if not self.move_arm_to_target(pick_pose.position.x, 
                                         pick_pose.position.y, 
                                         pick_pose.position.z + 0.1):
                return False
            
            # Open gripper
            if not self.control_gripper(False):
                return False
            
            # Lower to grasp
            if not self.move_arm_to_target(pick_pose.position.x, 
                                         pick_pose.position.y, 
                                         pick_pose.position.z):
                return False
            
            # Close gripper
            if not self.control_gripper(True):
                return False
            
            # Lift
            return self.move_arm_to_target(pick_pose.position.x, 
                                         pick_pose.position.y, 
                                         pick_pose.position.z + 0.15)
            
        except Exception as e:
            self.get_logger().error(f'Pick operation failed: {e}')
            return False

    def return_to_bin(self):
        """Return to bin and drop the dice"""
        if not self.navigate_sync(BIN_LOCATION['x'], BIN_LOCATION['y']):
            return False
        
        try:
            # Move above bin
            if not self.move_arm_to_target(0.3, 0.0, 0.4):
                return False
                
            # Lower into bin
            if not self.move_arm_to_target(0.3, 0.0, 0.2):
                return False
            
            # Open gripper and wait for drop
            if not self.control_gripper(False):
                return False
            time.sleep(0.5)
            
            # Retract arm
            if not self.move_arm_to_target(0.3, 0.0, 0.4):
                return False
                
            # Return to home
            if not self.move_to_home_position():
                return False
                
            self.detected_dice = None
            return True
                
        except Exception as e:
            self.get_logger().error(f'Place operation failed: {e}')
            return False

    def timer_callback(self):
        if self.is_busy or self.current_nav_goal is not None:
            return

        try:
            self.is_busy = True
            
            if self.detected_dice:
                self.get_logger().info('Attempting to pick dice')
                if self.pick_dice():
                    self.return_to_bin()
                self.detected_dice = None
            else:
                # Continue exploration
                waypoint = WAYPOINTS[self.current_waypoint]
                self.get_logger().info(f'Moving to waypoint: {waypoint}')
                
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