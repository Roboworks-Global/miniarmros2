import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, BoundingBox3D
from cv_bridge import CvBridge

class ColorDetectorNode(Node):
    COLOUR_CODES = {
        "blue": (255, 0, 0),
    }

    BOX_SIZE = 40  # size of the box in pixels
    BOX_SPACING = 100  # spacing between boxes in pixels
    BOX_COUNT = 3  # number of boxes

    def __init__(self):
        super().__init__('color_detector_node')  # Updated the node name here
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)

        self.detections_pub = self.create_publisher(Detection3DArray, 'detections', 10)

        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None

        self.timer = self.create_timer(0.05, self.timer_callback)

    def image_callback(self, msg: Image):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def create_bounding_boxes(self):
        boxes = []
        height, width = self.rgb_image.shape[:2]
        
        # Calculate box positions
        for i in range(self.BOX_COUNT):
            box_x = (width // 2) + (i - 1) * self.BOX_SPACING
            box_y = height // 2
            boxes.append((box_x, box_y))

        return boxes

    def convert_bb_to_3d(self, box_x, box_y):
        # Use the center pixel in the depth image to get z value
        z = self.depth_image[box_y, box_x] / 1000.0  # Convert to meters
        return z

    def timer_callback(self):
        if self.rgb_image is None or self.depth_image is None or self.camera_info is None:
            return

        boxes = self.create_bounding_boxes()
        detections_msg = Detection3DArray()
        detections_msg.header = self.camera_info.header

        for (box_x, box_y) in boxes:
            # Draw the box
            cv2.rectangle(self.rgb_image, (box_x - self.BOX_SIZE // 2, box_y - self.BOX_SIZE // 2),
                          (box_x + self.BOX_SIZE // 2, box_y + self.BOX_SIZE // 2),
                          self.COLOUR_CODES['blue'], thickness=3)

            # Create 3D bounding box
            z = self.convert_bb_to_3d(box_x, box_y)
            bbox3d = BoundingBox3D()
            bbox3d.center.position.x = box_x  # Placeholder for x
            bbox3d.center.position.y = box_y  # Placeholder for y
            bbox3d.center.position.z = z
            bbox3d.size.x = self.BOX_SIZE / 1000.0  # Convert to meters for size
            bbox3d.size.y = self.BOX_SIZE / 1000.0
            bbox3d.size.z = z  # Use z as height for simplicity

            detection = Detection3D()
            detection.bbox = bbox3d
            detections_msg.detections.append(detection)

            # Display Z value
            cv2.putText(self.rgb_image, f'Z: {z:.2f} m', (box_x - self.BOX_SIZE // 2, box_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLOUR_CODES['blue'], 1)

        self.detections_pub.publish(detections_msg)
        cv2.imshow('Camera Feed', self.rgb_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    color_detector_node = ColorDetectorNode()
    rclpy.spin(color_detector_node)
    cv2.destroyAllWindows()
    color_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
