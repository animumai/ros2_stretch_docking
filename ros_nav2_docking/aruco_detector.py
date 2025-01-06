import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
from cv2 import aruco
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.marker_length = self.declare_parameter('marker_length', 0.11).value  # Marker size in meters
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_link').value
        self.aruco_dictionary_name = self.declare_parameter('aruco_dictionary', 'DICT_4X4_50').value

        # ArUco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, self.aruco_dictionary_name))
        self.aruco_params = aruco.DetectorParameters_create()

        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None

        # CV bridge
        self.bridge = CvBridge()

        # Publisher for the docking station pose
        self.pose_pub = self.create_publisher(PoseStamped, 'dock_pose', 10)

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        self.get_logger().info("Aruco Detector Node Initialized")

    def camera_info_callback(self, msg):
        """Callback to set camera intrinsics from CameraInfo topic."""
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        """Callback to process images and detect ArUco markers."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warning("Camera intrinsics not received yet.")
            return

        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Estimate pose for each marker
            for i, corner in enumerate(corners):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corner, self.marker_length, self.camera_matrix, self.dist_coeffs
                )

                # Publish the pose of the detected marker
                self.publish_pose(ids[i][0], tvec[0][0], rvec[0][0])

                # Draw the marker and axis for visualization
                aruco.drawDetectedMarkers(frame, corners, ids)
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], 0.1)

        # For debugging, display the image
        cv2.imshow('Aruco Detection', frame)
        cv2.waitKey(1)

    def publish_pose(self, marker_id, translation, rotation):
        """Publish the detected marker's pose."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.camera_frame

        # Translation
        pose.pose.position.x = translation[0]
        pose.pose.position.y = translation[1]
        pose.pose.position.z = translation[2]

        # Rotation (convert from rotation vector to quaternion)
        quaternion = self.rvec_to_quaternion(rotation)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        self.pose_pub.publish(pose)
        self.get_logger().info(f"Published pose for marker ID {marker_id}")

    @staticmethod
    def rvec_to_quaternion(rvec):
        """Convert rotation vector to quaternion."""
        theta = np.linalg.norm(rvec)
        if theta < 1e-6:
            return [0.0, 0.0, 0.0, 1.0]

        axis = rvec / theta
        q = [
            axis[0] * np.sin(theta / 2),
            axis[1] * np.sin(theta / 2),
            axis[2] * np.sin(theta / 2),
            np.cos(theta / 2)
        ]
        return q


def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()