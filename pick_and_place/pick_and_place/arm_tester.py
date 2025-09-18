#!/usr/bin/env python3
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


class ArmTester(Node):
    def __init__(self):
        super().__init__('arm_tester')
        
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
    
    def create_pick_pose(self, target_pose):
        """Convert target pose to arm coordinates"""
        # Note: You'll need to transform the target pose from map frame 
        # to arm base frame before using these coordinates
        try:
            transform = self.tf_buffer.lookup_transform(
                'arm_base_link',  # Adjust this arm's base frame
                target_pose.header.frame_id,
                rclpy.time.Time()
            )
            transformed_pose = tf2_geometry_msgs.do_transform_pose(target_pose, transform)
            return transformed_pose.pose
            
        except Exception as e:
            self.get_logger().error(f'Failed to transform pick pose: {e}')
            return None

    def move_to_home_position(self):
        """Move the arm to its home position"""
        return self.move_arm_to_target(0.3, 0.0, 0.4)  # Adjust these values based on you

    def pick_dice(self):
        """Pick up the detected dice"""
        try:
            
            pick_pose = self.create_pick_pose(self.detected_dice)
            self.get_logger().info('flag 1')
            if not pick_pose:
                self.get_logger().info('flag 2')
                return False
            
            # Pre-grasp
            self.get_logger().info('flag 3')
            if not self.move_arm_to_target(pick_pose.position.x, 
                                         pick_pose.position.y, 
                                         pick_pose.position.z + 0.1):
                self.get_logger().info('flag 4')
                return False
            
            # Open gripper
            self.get_logger().info('flag 5')
            if not self.control_gripper(False):
                self.get_logger().info('flag 6')
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


def main():
    rclpy.init()
    arm  = ArmTester()
    
    arm.control_gripper(True)
    time.sleep(2.0)
    arm.control_gripper(False)
    time.sleep(2.0)
    arm.control_gripper(True)
    time.sleep(2.0)
    arm.control_gripper(False)
    
    # arm.pick_dice()

    try:
        rclpy.spin(arm)
    except KeyboardInterrupt:
        pass
    finally:
        arm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()