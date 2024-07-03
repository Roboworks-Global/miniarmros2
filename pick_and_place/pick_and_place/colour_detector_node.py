import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, BoundingBox3D, BoundingBox2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from typing import List, Tuple

# Global variables to store HSV ranges for multiple colors

COLOUR_CODES = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "yellow": (0, 255, 255),
}


class ColorDetectorNode(Node):
    hsv_ranges = {
        "green": [],
        "blue": [],
        "yellow": [],
        "red": [],
    }

    depth_image_units_divisor = 1000.0
    current_color = None
    rgb_image = None
    depth_image = None
    camera_info = None
    maximum_detection_threshold = 0.3
    min_contour_area = 200
    blur_kernel_size = 3
    target_frame = "camera_link"

    H_TOLERANCE = 10
    S_TOLERANCE = 20
    V_TOLERANCE = 20

    def __init__(self):
        super().__init__('color_detector')
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)

        self.detections_pub = self.create_publisher(Detection3DArray, 'detections', 10)

        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        cv2.namedWindow('Mask')
        cv2.namedWindow('Camera Feed')
        cv2.setMouseCallback('Camera Feed', self.get_hsv)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def image_callback(self, msg : Image):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def camera_info_callback(self, msg):
        self.camera_info = msg


    def get_hsv(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.current_color is not None:
            bgr_pixel = self.rgb_image[y, x]
            hsv_pixel = cv2.cvtColor(np.uint8([[bgr_pixel]]), cv2.COLOR_BGR2HSV)[0][0]

            hsv_range = (
                np.array([
                    max(hsv_pixel[0] - self.H_TOLERANCE, 0),
                    max(hsv_pixel[1] - self.S_TOLERANCE, 0),
                    max(hsv_pixel[2] - self.V_TOLERANCE, 0)]),
                np.array([
                    min(hsv_pixel[0] + self.H_TOLERANCE, 179),
                    min(hsv_pixel[1] + self.S_TOLERANCE, 255),
                    min(hsv_pixel[2] + self.V_TOLERANCE, 255)])
            )

            self.hsv_ranges[self.current_color].append(hsv_range)
            self.get_logger().info(f"Clicked Pixel at ({x}, {y}) - BGR: {bgr_pixel}, HSV: {hsv_pixel}")
            self.get_logger().info(f"Added HSV Range: {hsv_range} to {self.current_color}")

            self.current_color = None

    def select_color(self, key):
        if key == ord('g'):
            self.current_color = "green"
            self.get_logger().info(f"Selected Color: {self.current_color}") 
        elif key == ord('b'):
            self.current_color = "blue"
            self.get_logger().info(f"Selected Color: {self.current_color}") 
        elif key == ord('r'):
            self.current_color = "red"
            self.get_logger().info(f"Selected Color: {self.current_color}") 
        elif key == ord('y'):
            self.current_color = "yellow"
            self.get_logger().info(f"Selected Color: {self.current_color}")

    def convert_bb_to_3d(
        self,
        bbox: BoundingBox2D,
    ) -> BoundingBox3D:

        # crop depth image by the 2d BB
        center_x = int(bbox.center.position.x)
        center_y = int(bbox.center.position.y)
        size_x = int(bbox.size_x)
        size_y = int(bbox.size_y)

        u_min = max(center_x - size_x // 2, 0)
        u_max = min(center_x + size_x // 2, self.depth_image.shape[1] - 1)
        v_min = max(center_y - size_y // 2, 0)
        v_max = min(center_y + size_y // 2, self.depth_image.shape[0] - 1)

        roi = self.depth_image[v_min:v_max, u_min:u_max] / \
            self.depth_image_units_divisor  # convert to meters
        
        if not np.any(roi):
            return None

        # find the z coordinate on the 3D BB
        bb_center_z_coord = self.depth_image[int(center_y)][int(
            center_x)] / self.depth_image_units_divisor
        z_diff = np.abs(roi - bb_center_z_coord)
        mask_z = z_diff <= self.maximum_detection_threshold
        if not np.any(mask_z):
            return None

        roi_threshold = roi[mask_z]
        z_min, z_max = np.min(roi_threshold), np.max(roi_threshold)
        z = (z_max + z_min) / 2
        if z == 0:
            return None

        # project from image to world space
        k = self.camera_info.k
        px, py, fx, fy = k[2], k[5], k[0], k[4]
        x = z * (center_x - px) / fx
        y = z * (center_y - py) / fy
        w = z * (size_x / fx)
        h = z * (size_y / fy)

        # create 3D BB
        msg = BoundingBox3D()
        msg.center.position.x = x
        msg.center.position.y = y
        msg.center.position.z = z
        msg.size.x = w
        msg.size.y = h
        msg.size.z = float(z_max - z_min)

        return msg

    def get_transform(self, frame_id: str) -> Tuple[np.ndarray]:
        # transform position from image frame to target_frame
        rotation = None
        translation = None

        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame_id,
                rclpy.time.Time())

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


    def timer_callback(self):
        if self.rgb_image is None or self.depth_image is None or self.camera_info is None:
            return

        if not self.hsv_ranges:
            cv2.imshow("Camera Feed", self.rgb_image)
            return
        
        transform = self.get_transform(self.camera_info.header.frame_id)
        if transform is None:
            return

        key = cv2.waitKey(1)
        self.select_color(key)

        kernel = np.ones((self.blur_kernel_size, self.blur_kernel_size), np.float32) / self.blur_kernel_size ** 2
        processed_image = cv2.filter2D(self.rgb_image, -1, kernel)

        detected_cubes = self.rgb_image.copy()
        combined_combined_mask = np.zeros(self.rgb_image.shape, dtype=np.uint8)

        detections_msg = Detection3DArray()
        detections_msg.header = self.camera_info.header

        for colour in self.hsv_ranges.keys():
            combined_mask = np.zeros(processed_image.shape[:2], dtype=np.uint8)
            for hsv_range in self.hsv_ranges[colour]:
                hsv_frame = cv2.cvtColor(processed_image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv_frame, hsv_range[0], hsv_range[1])
                combined_mask = cv2.bitwise_or(combined_mask, mask)

            contours, _ = cv2.findContours(combined_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            mask_with_contours = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(mask_with_contours, contours, -1, COLOUR_CODES[colour], 2)
            combined_combined_mask = cv2.bitwise_or(combined_combined_mask, mask_with_contours)

            for contour in contours:
                if cv2.contourArea(contour) > self.min_contour_area:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(detected_cubes, (x, y), (x + w, y + h), COLOUR_CODES[colour], 2)

                    bbox2d = BoundingBox2D()
                    bbox2d.center.position.x = float(x + w // 2)
                    bbox2d.center.position.y = float(y + h // 2)
                    bbox2d.size_x = float(w)
                    bbox2d.size_y = float(h)

                    bbox3d = self.convert_bb_to_3d(bbox2d)

                    if bbox3d is not None:
                        bbox3d = self.transform_3d_box(bbox3d, transform[0], transform[1])
                        label = f'X: {bbox3d.center.position.x:.2f}, Y: {bbox3d.center.position.y:.2f}, Z: {bbox3d.center.position.z:.2f}'
                        cv2.putText(detected_cubes, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOUR_CODES[colour], 2)

                        detection = Detection3D()
                        detection.bbox = bbox3d
                        detection.id = colour
                        detections_msg.detections.append(detection)


        combined_combined_mask = cv2.addWeighted(processed_image, 0.7, combined_combined_mask, 0.3, 0)
        self.detections_pub.publish(detections_msg)
        cv2.imshow('Mask', combined_combined_mask)
        cv2.imshow('Camera Feed', detected_cubes)

    def transform_3d_box(
        self,
        bbox: BoundingBox3D,
        translation: np.ndarray,
        rotation: np.ndarray
    ) -> BoundingBox3D:

        # position
        position = self.qv_mult(
            rotation,
            np.array([bbox.center.position.x,
                      bbox.center.position.y,
                      bbox.center.position.z])
        ) + translation

        bbox.center.position.x = position[0]
        bbox.center.position.y = position[1]
        bbox.center.position.z = position[2]

        # size
        size = self.qv_mult(
            rotation,
            np.array([bbox.size.x,
                      bbox.size.y,
                      bbox.size.z])
        )

        bbox.size.x = abs(size[0])
        bbox.size.y = abs(size[1])
        bbox.size.z = abs(size[2])

        return bbox

    def qv_mult(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        q = np.array(q, dtype=np.float64)
        v = np.array(v, dtype=np.float64)
        qvec = q[1:]
        uv = np.cross(qvec, v)
        uuv = np.cross(qvec, uv)
        return v + 2 * (uv * q[0] + uuv)


def main(args=None):
    rclpy.init(args=args)
    color_detector_node = ColorDetectorNode()
    rclpy.spin(color_detector_node)
    color_detector_node.cap.release()
    cv2.destroyAllWindows()
    color_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
