#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray
from control_msgs.msg import GripperCommand
import time

from control_msgs.msg import GripperCommand
from moveit_msgs.msg import MoveItErrorCodes
from moveit.planning import MoveGroupInterface, PlanningSceneInterface
import time
import tf2_ros
import tf2_geometry_msgs

from tf2_ros import Buffer, TransformListener, TransformBroadcaster


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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Vision subscriber
        self.detected_dices = []
        self.detected_dice = None
        self.detection_sub = self.create_subscription(
            Detection3DArray,
            'detections',
            self.detection_callback,
            10
        )
        
        # Initialize MoveIt
        self.move_group_arm = MoveGroupInterface("arm", "robot_description")
        self.move_group_hand = MoveGroupInterface("hand", "robot_description")
        self.scene = PlanningSceneInterface()
        
        # Set movement parameters
        self.move_group_arm.set_max_velocity_scaling_factor(0.5)
        self.move_group_arm.set_max_acceleration_scaling_factor(0.5)
        self.move_group_arm.set_goal_orientation_tolerance(0.1)
        self.move_group_arm.set_goal_position_tolerance(0.01)

        # State management
        self.current_waypoint = 0
        self.is_busy = False
        self.current_nav_goal = None  # Store current navigation goal handle
        
        # Create timer for exploration
        self.timer = self.create_timer(1.0, self.timer_callback)
    def control_gripper(self, open_width):
        """
        Control gripper using MoveIt
        @param open_width: float between 0.0 (closed) and 0.08 (open)
        """
        try:
            # Scale the open_width to joint values for all gripper joints
            joint_values = {}
            if open_width == GRIPPER_OPEN:  # 0.08
                # Open position
                joint_values = {
                    'hand_left_joint': 0.04,
                    'hand_left2_joint': 0.04,
                    'hand_right_joint': -0.04,
                    'hand_right2_joint': -0.04
                }
            else:  # GRIPPER_CLOSED (0.02)
                # Closed position
                joint_values = {
                    'hand_left_joint': 0.01,
                    'hand_left2_joint': 0.01,
                    'hand_right_joint': -0.01,
                    'hand_right2_joint': -0.01
                }

            # Set joint values
            self.move_group_hand.set_joint_value_target(joint_values)
            
            # Plan and execute
            success = self.move_group_hand.go(wait=True)
            self.move_group_hand.stop()
            
            if not success:
                self.get_logger().error('Failed to move gripper')
                return False
                
            return True
            
        except Exception as e:
            self.get_logger().error(f'Gripper control failed: {e}')
            return False
        
    def move_to_home_position(self):
        """Move the arm to its home position"""
        self.move_group_arm.set_named_target("home")
        success = self.move_group_arm.go(wait=True)
        self.move_group_arm.stop()
        return success
    
    def create_pick_pose(self, target_pose):
        """Create picking pose from detected dice position"""
        pick_pose = PoseStamped()
        pick_pose.header.frame_id = "map"
        pick_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position slightly above the dice
        pick_pose.pose.position.x = target_pose.position.x
        pick_pose.pose.position.y = target_pose.position.y
        pick_pose.pose.position.z = target_pose.position.z + 0.1  # Approach from above
        
        # Set orientation for vertical approach
        pick_pose.pose.orientation.x = 0.0
        pick_pose.pose.orientation.y = 0.707
        pick_pose.pose.orientation.z = 0.0
        pick_pose.pose.orientation.w = 0.707
        
        return pick_pose
    
    def detection_callback(self, msg):
        if self.is_busy:
            return
        try:
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f'Failed to lookup transform: {e}')
            return
        for detection in msg.detections:
            # transform detection to map frame
            try:
                detection_map_coordinates= tf2_geometry_msgs.do_transform_pose(detection.bbox.center, transform)
            except Exception as e:
                self.get_logger().error(f'Failed to transform detection: {e}')
                continue
            is_duplicate = False
            for existing_dice in self.detected_dices:
                if abs(existing_dice.position.y - detection_map_coordinates.position.y) < 0.3 and abs(existing_dice.position.x - detectionMap.position.x)<0.3: # if the dice is close to each other, it is probably the same dice seen from different angles before.
                    is_duplicate = True
                    break
            if is_duplicate: continue
            self.detected_dices.append(detection_map_coordinates)
        self.get_logger().info('Red dice detected!')
        # Cancel current navigation if exploring
        if self.current_nav_goal is not None:
            self.current_nav_goal.cancel_goal_async()
            self.current_nav_goal = None


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
        target = self.detected_dice.position
        if not self.navigate_sync(target.x - APPROACH_DISTANCE, target.y):
            return False
        
        try:
            # Navigate to approach pose
            target = self.detected_dice.position
            if not self.navigate_sync(target.x - APPROACH_DISTANCE, target.y):
                return False
            
            # Create pick pose
            pick_pose = self.create_pick_pose(self.detected_dice)
            
            # Pre-grasp position
            pick_pose.pose.position.z += 0.1  # Higher approach position
            self.move_group_arm.set_pose_target(pick_pose)
            if not self.move_group_arm.go(wait=True):
                self.get_logger().error('Failed to reach pre-grasp position')
                return False
            
            # Open gripper using MoveIt
            if not self.open_gripper():
                return False
            
            # Move down to grasp
            pick_pose.pose.position.z -= 0.1
            self.move_group_arm.set_pose_target(pick_pose)
            if not self.move_group_arm.go(wait=True):
                self.get_logger().error('Failed to reach grasp position')
                return False
            
            # Close gripper using MoveIt
            if not self.close_gripper():
                return False
            
            # Lift object
            pick_pose.pose.position.z += 0.15
            self.move_group_arm.set_pose_target(pick_pose)
            success = self.move_group_arm.go(wait=True)
            
            if not success:
                self.get_logger().error('Failed to lift object')
                return False
                
            return True
            
        except Exception as e:
            self.get_logger().error(f'Pick operation failed: {e}')
            return False
        finally:
            self.move_group_arm.stop()
            self.move_group_arm.clear_pose_targets()

    def return_to_bin(self):
        # Navigate to bin (synchronized)
        if not self.navigate_sync(BIN_LOCATION['x'], BIN_LOCATION['y']):
            return False
        
        try:
            # Create place pose
            place_pose = PoseStamped()
            place_pose.header.frame_id = "map"
            place_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Position above bin
            place_pose.pose.position.x = BIN_LOCATION['x']
            place_pose.pose.position.y = BIN_LOCATION['y']
            place_pose.pose.position.z = 0.4  # Higher position above bin
            
            # Orientation for dropping (vertical orientation)
            place_pose.pose.orientation.x = 0.0
            place_pose.pose.orientation.y = 0.707
            place_pose.pose.orientation.z = 0.0
            place_pose.pose.orientation.w = 0.707
            
            # Move arm to pre-place position
            self.move_group_arm.set_pose_target(place_pose)
            if not self.move_group_arm.go(wait=True):
                self.get_logger().error('Failed to move to pre-place position')
                return False
                
            # Lower into bin
            place_pose.pose.position.z = 0.2  # Lower position inside bin
            self.move_group_arm.set_pose_target(place_pose)
            if not self.move_group_arm.go(wait=True):
                self.get_logger().error('Failed to move to place position')
                return False
            
            # Open gripper using MoveIt
            if not self.open_gripper():
                self.get_logger().error('Failed to open gripper')
                return False
                
            # Slight pause to ensure object drops
            time.sleep(0.5)
            
            # Move arm up
            place_pose.pose.position.z = 0.4
            self.move_group_arm.set_pose_target(place_pose)
            if not self.move_group_arm.go(wait=True):
                self.get_logger().error('Failed to retract arm')
                return False
                
            # Return to home position
            if not self.move_to_home_position():
                self.get_logger().error('Failed to return to home position')
                return False
                
            self.detected_dice = None
            return True
                
        except Exception as e:
            self.get_logger().error(f'Place operation failed: {e}')
            return False
        finally:
            self.move_group_arm.stop()
            self.move_group_arm.clear_pose_targets()

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