import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from cv_bridge import CvBridge
import cv2
import math
import numpy as np

import tf2_ros
import tf2_geometry_msgs  # needed for do_transform_pose()

class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__('aruco_detect_node')

        # Parameters
        self.declare_parameter('marker_id', 245)
        self.declare_parameter('marker_size_meters', 0.11)  # 110 mm
        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size_meters').value

        # Publishers
        self.aruco_pose_raw_pub = self.create_publisher(
            PoseStamped, '/detected_aruco', 10
        )
        self.dock_pose_pub = self.create_publisher(
            PoseStamped, '/detected_dock_pose', 10
        )

        # TF2 buffer/listener to transform camera frame to base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera subscriptions
        self.subscription_image = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.subscription_camera_info = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.info_callback, 10
        )

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # ArUco dictionaries
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.get_logger().info('ArucoDetectNode started.')

    def info_callback(self, info_msg):
        # Extract camera intrinsics
        self.camera_matrix = np.array(info_msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(info_msg.d)
        # Use destroy_subscription to read camera info only once
        self.destroy_subscription(self.subscription_camera_info)
        self.subscription_camera_info = None
        self.get_logger().info('Camera info received and stored.')

    def image_callback(self, img_msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return  # wait for camera intrinsics

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv_image,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is None:
            return

        # Check if our target marker is found
        ids = ids.flatten()
        if self.marker_id not in ids:
            return

        idx = list(ids).index(self.marker_id)
        # Estimate the pose of the single marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners[idx],
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs
        )
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]

        # Convert rvec to rotation matrix
        rot_matrix, _ = cv2.Rodrigues(rvec)
        
        # Compute the quaternion from the rotation matrix
        quat = self.rotation_matrix_to_quaternion(rot_matrix)

        # Create the PoseStamped in camera optical frame
        pose_msg_raw = PoseStamped()
        pose_msg_raw.header.stamp = self.get_clock().now().to_msg()
        pose_msg_raw.header.frame_id = 'camera_color_optical_frame'
        pose_msg_raw.pose = Pose(
            position=Point(x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2])),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        )

        # Publish the raw pose for debugging
        self.aruco_pose_raw_pub.publish(pose_msg_raw)

        # Try to transform this pose into base_link
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                'base_link',
                pose_msg_raw.header.frame_id,
                rclpy.time.Time()
            )

            # Instead of passing a PoseStamped (with header from camera),
            # pass only the inner Pose to transform_pose, then rebuild the PoseStamped:
            raw_pose = pose_msg_raw.pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose(raw_pose, transform_stamped)
            
            # Create a new PoseStamped message with the correct header
            pose_in_base = PoseStamped()
            # Use the current time for the new stamp and set the frame_id to 'base_link'
            pose_in_base.header.stamp = self.get_clock().now().to_msg()
            pose_in_base.header.frame_id = 'base_link'
            pose_in_base.pose = transformed_pose

            self.dock_pose_pub.publish(pose_in_base)

        except Exception as ex:
            self.get_logger().warn(f"TF transform error: {ex}")

    def rotation_matrix_to_quaternion(self, R):
        # Defensive approach to avoid sqrt(negative)
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        val = 1.0 + trace
        if val < 0.0:
            val = 0.0
        if val > 4.0:
            val = 4.0

        qw = math.sqrt(val) / 2.0
        if abs(qw) < 1e-8:
            return (0.0, 0.0, 0.0, 1.0)

        qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)

        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()