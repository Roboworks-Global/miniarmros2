#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
import cv2
import numpy as np
from cv_bridge import CvBridge
import math
from tf2_ros import Buffer, TransformListener, TransformException
from typing import Tuple

class SimpleColorDetector(Node):
    """
    A simple ROS2 node to detect a colored block using a RGB-D camera
    and publish its 3D position in the map frame.
    """
    
    def __init__(self):
        super().__init__('simple_color_detector')
        
        # ROS2 Subscriptions
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.info_callback, 10)
        
        # ROS2 Publisher for the detected object's position
        self.position_pub = self.create_publisher(PointStamped, 'detected_object_position', 10)

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = "map"
        
        # Class variables to store the latest image data and camera info
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        
        # HSV range for the target color.
        self.hsv_range = None
        
        # Minimum contour area to filter for blocks
        self.min_contour_area = 500
        
        # OpenCV Bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # User interaction setup for defining the color range
        cv2.namedWindow('Camera Feed')
        cv2.setMouseCallback('Camera Feed', self.get_hsv_range)
        self.get_logger().info("Click on the target color in the 'Camera Feed' window to set the detection range.")
        self.clicked_points = []
        
        # Main processing loop
        self.timer = self.create_timer(0.1, self.timer_callback)

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        
    def info_callback(self, msg):
        self.camera_info = msg
    
    def get_hsv_range(self, event, x, y, flags, param):
        """
        Callback function for mouse clicks on the OpenCV window.
        Samples HSV values from the clicked pixel to build a color range.
        """
        if event == cv2.EVENT_LBUTTONDOWN and self.rgb_image is not None:
            # Get the BGR color of the pixel clicked
            bgr_pixel = self.rgb_image[y, x]
            # Convert the BGR pixel to HSV
            hsv_pixel = cv2.cvtColor(np.uint8([[bgr_pixel]]), cv2.COLOR_BGR2HSV)[0][0]
            
            self.clicked_points.append(hsv_pixel)
            self.get_logger().info(f"Sampled HSV point: {hsv_pixel}")
            
            if len(self.clicked_points) >= 1:
                # Calculate the average HSV value
                avg_hsv = np.mean(self.clicked_points, axis=0)
                
                # Define a small tolerance for the range
                tolerance = np.array([10, 30, 30])
                
                lower_bound = np.array([max(0, avg_hsv[0] - tolerance[0]),
                                        max(0, avg_hsv[1] - tolerance[1]),
                                        max(0, avg_hsv[2] - tolerance[2])], dtype=np.uint8)
                
                upper_bound = np.array([min(179, avg_hsv[0] + tolerance[0]),
                                        min(255, avg_hsv[1] + tolerance[1]),
                                        min(255, avg_hsv[2] + tolerance[2])], dtype=np.uint8)
                
                self.hsv_range = (lower_bound, upper_bound)
                self.get_logger().info(f"Updated HSV Range: {self.hsv_range}")
                self.clicked_points = [] # Reset points after defining the range


    def get_transform(self, frame_id: str) -> Tuple[np.ndarray]:
        """
        Looks up the transform from the camera frame to the target frame.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame_id,
                rclpy.time.Time(), # grab the latest tf.
            )
            translation = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z])
            rotation = np.array([transform.transform.rotation.w,
                                 transform.transform.rotation.x,
                                 transform.transform.rotation.y,
                                 transform.transform.rotation.z])
            return translation, rotation
        except TransformException as ex:
            self.get_logger().error(f"Could not transform: {ex}")
            return None

    def transform_point_to_map(self, point: np.ndarray, translation: np.ndarray, rotation: np.ndarray) -> np.ndarray:
        """
        Transforms a 3D point from the camera frame to the map frame.
        """
        # Quaternion-vector multiplication
        q_vec = rotation[1:]
        uv = np.cross(q_vec, point)
        uuv = np.cross(q_vec, uv)
        transformed_point = point + 2 * (rotation[0] * uv + uuv)
        
        # Add the translation
        return transformed_point + translation


    def timer_callback(self):
        """
        Main processing loop that runs at a fixed frequency.
        """
        if self.rgb_image is None or self.depth_image is None or self.camera_info is None or self.hsv_range is None:
            # If any data is missing or a color range is not set, return
            return

        # Get the transform from the camera frame to the map frame
        transform = self.get_transform(self.camera_info.header.frame_id)
        if transform is None:
            self.get_logger().warn("TF not available.")
            return

        # Convert RGB to HSV color space
        hsv_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
        
        # Create a binary mask for the specified HSV range
        mask = cv2.inRange(hsv_image, self.hsv_range[0], self.hsv_range[1])
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # If no contours are found, show the image and return
        if not contours:
            cv2.imshow('Camera Feed', self.rgb_image)
            cv2.imshow('Mask', mask)
            cv2.waitKey(1)
            return

        # Find the contour with the largest area (or closest depth, for a flat surface)
        closest_contour = None
        min_depth = float('inf')
        
        for contour in contours:
            # Filter by area to find "blocks" and not just noise
            if cv2.contourArea(contour) < self.min_contour_area:
                continue
            
            # Find the center of the contour to get the depth
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Get depth value at the center pixel
                depth_at_center = self.depth_image[cy, cx] / 1000.0 # Convert to meters
                
                if depth_at_center > 0 and depth_at_center < min_depth:
                    min_depth = depth_at_center
                    closest_contour = contour
                    
        # If no suitable contour was found
        if closest_contour is None:
            cv2.imshow('Camera Feed', self.rgb_image)
            cv2.imshow('Mask', mask)
            cv2.waitKey(1)
            return
            
        # Draw a bounding box around the closest object
        x, y, w, h = cv2.boundingRect(closest_contour)
        cv2.rectangle(self.rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Calculate the 3D position of the object in the camera frame
        # Using the camera's intrinsic parameters (K matrix)
        k = self.camera_info.k
        fx = k[0]
        fy = k[4]
        cx = k[2]
        cy = k[5]
        
        # Center of the 2D bounding box
        u = x + w / 2
        v = y + h / 2
        
        # Convert to 3D coordinates in the camera frame
        z = min_depth
        point_x_camera = (u - cx) * z / fx
        point_y_camera = (v - cy) * z / fy

        # Transform the point to the map frame
        transformed_point = self.transform_point_to_map(
            np.array([point_x_camera, point_y_camera, z]),
            transform[0],
            transform[1]
        )
        
        # Publish the 3D position
        position_msg = PointStamped()
        position_msg.header.stamp = self.get_clock().now().to_msg()
        position_msg.header.frame_id = self.target_frame
        position_msg.point.x = float(transformed_point[0])
        position_msg.point.y = float(transformed_point[1])
        position_msg.point.z = float(transformed_point[2])
        self.position_pub.publish(position_msg)
        self.get_logger().info(f"Detected object at: ({transformed_point[0]:.2f}, {transformed_point[1]:.2f}, {transformed_point[2]:.2f}) m in '{self.target_frame}' frame")
        
        # Show the images
        cv2.imshow('Camera Feed', self.rgb_image)
        cv2.imshow('Mask', mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    detector = SimpleColorDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
