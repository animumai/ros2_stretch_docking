import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import math
from math import sin, cos

class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')

        # Declare parameters
        self.declare_parameter('map_yaml_file', '')
        self.declare_parameter('dock_pose', [0.0, 0.0, 0.0])  # [x, y, theta] in map
        self.declare_parameter('staging_offset', [-0.7, 0.0])
        self.declare_parameter('external_detection_offsets', [0.0, 0.0, 0.0])
        
        self.map_yaml_file = self.get_parameter('map_yaml_file').value
        self.dock_pose_xytheta = self.get_parameter('dock_pose').value
        self.staging_offset = self.get_parameter('staging_offset').value
        self.external_detection_offsets = self.get_parameter('external_detection_offsets').value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pose_pub = self.create_publisher(Float64MultiArray, '/joint_pose_cmd', 10)

        # Subscription to the *base_link* pose from ArUco detection
        self.dock_pose_sub = self.create_subscription(
            PoseStamped,
            '/detected_dock_pose',
            self.aruco_pose_callback,
            10
        )

        # Nav2 interface
        self.navigator = BasicNavigator()

        # Internal states
        self.docking_in_progress = False
        self.visual_servoing = False

        # Track last time marker was seen
        self.last_detection_time_ = None

        # Create a timer to check "marker lost"
        self.lost_timer_ = self.create_timer(1.0, self.check_marker_lost)

        self.get_logger().info('Docking Node started.')

    def start_docking(self):
        if self.docking_in_progress:
            self.get_logger().warn('Docking already in progress.')
            return

        self.move_camera_down_45_and_rotate_180()
        self.docking_in_progress = True

        # (A) Compute staging pose in the map
        staging_pose_stamped = self.compute_staging_pose()

        # (B) Wait until Nav2 is active, then navigate
        self.navigator.waitUntilNav2Active()
        self.navigator.goToPose(staging_pose_stamped)

        # (C) Create a timer to monitor the Nav2 result
        self.monitor_timer = self.create_timer(0.5, self.monitor_navigation_result)

        self.get_logger().info('Docking started. Navigating to staging pose...')

    def compute_staging_pose(self):
        # Build a PoseStamped for the staging location
        staging_pose = PoseStamped()
        staging_pose.header.frame_id = 'map'
        staging_pose.header.stamp = self.get_clock().now().to_msg()

        dock_x, dock_y, dock_theta = self.dock_pose_xytheta
        offset_x, offset_y = self.staging_offset

        # Calculate the staging position
        staging_x = dock_x - offset_x * cos(dock_theta) + offset_y * sin(dock_theta)
        staging_y = dock_y - offset_x * sin(dock_theta) - offset_y * cos(dock_theta)
        staging_theta = dock_theta

        # Convert to quaternion
        qz = math.sin(staging_theta / 2.0)
        qw = math.cos(staging_theta / 2.0)

        staging_pose.pose.position.x = staging_x
        staging_pose.pose.position.y = staging_y
        staging_pose.pose.orientation.z = qz
        staging_pose.pose.orientation.w = qw

        return staging_pose

    def monitor_navigation_result(self):
        """
        Monitor the navigation result. If successful, start visual servoing.
        """
        # Check if navigation task is still in progress
        if not self.navigator.isTaskComplete():
            self.get_logger().debug('Still navigating to staging pose...')
            return

        # Once task is complete, get the result
        nav_result = self.navigator.getResult()
        self.get_logger().info(f'Navigation result: {nav_result}')

        if nav_result == TaskResult.SUCCEEDED:
            self.get_logger().info('Arrived at staging pose. Starting visual servoing.')
            self.visual_servoing = True
        elif nav_result == TaskResult.FAILED:
            self.get_logger().error('Navigation to staging pose failed. Resetting docking.')
            self.reset_and_start_docking()
        elif nav_result == TaskResult.CANCELED:
            self.get_logger().error('Navigation to staging pose was canceled. Resetting docking.')
            self.reset_and_start_docking()

        # Stop the timer since navigation is complete
        self.destroy_timer(self.monitor_timer)

    def aruco_pose_callback(self, msg: PoseStamped):
        """
        Called with the transformed pose (in base_link) from /detected_dock_pose.
        Handles the visual servoing loop.
        """
        self.last_detection_time_ = self.get_clock().now()

        if not self.visual_servoing:
            return

        # Desired offsets
        desired_x = self.external_detection_offsets[0]  # Forward offset
        desired_y = self.external_detection_offsets[1]        # Lateral offset

        # Tolerances for docking
        dx_tolerance = 0.05  # Forward tolerance (meters)
        dy_tolerance = 0.05  # Lateral tolerance (meters)

        # Compute errors
        dx = msg.pose.position.x - desired_x  # Forward/backward error
        dy = msg.pose.position.y - desired_y  # Left/right error

        # Initialize velocities
        linear_speed = 0.0
        angular_speed = 0.0

        # Forward motion control
        if abs(dx) > dx_tolerance:
            linear_speed = 0.2 * dx  # P-controller
            linear_speed = max(min(linear_speed, 0.1), -0.1)  # Clamp speed

        # Angular motion control
        if abs(dy) > dy_tolerance:
            angular_speed = -0.5 * dy  # P-controller for lateral correction
            angular_speed = max(min(angular_speed, 0.2), -0.2)  # Clamp speed

        # Stop motion if within tolerances
        if abs(dx) < dx_tolerance and abs(dy) < dy_tolerance:
            self.get_logger().info('Docking complete. Stopping robot.')
            self.visual_servoing = False
            self.docking_in_progress = False
            # Stop the robot
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            return

        # Publish velocity command
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)

        self.get_logger().debug(
            f"Visual servoing - dx: {dx:.3f}, dy: {dy:.3f}, "
            f"linear_speed: {linear_speed:.3f}, angular_speed: {angular_speed:.3f}"
        )


    def check_marker_lost(self):
        """
        Called every second by lost_timer_.
        If we are in visual_servoing but haven't seen the marker for >2.0s,
        then re-invoke the Nav2 navigation to staging.
        """
        if not self.visual_servoing:
            return  # Not actively servoing

        # If we've never seen the marker, last_detection_time_ is None => lost
        if self.last_detection_time_ is None:
            self.get_logger().warn('Marker never seen. Re-navigating to staging pose...')
            self.reset_and_start_docking()
            return

        # Check how long it has been since last detection
        elapsed = (self.get_clock().now() - self.last_detection_time_).nanoseconds * 1e-9
        if elapsed > 2.0:  # e.g., 2 seconds
            self.get_logger().warn('Marker lost for >2s. Re-navigating to staging pose...')
            self.reset_and_start_docking()

    def reset_and_start_docking(self):
        # Stop the servo loop
        self.visual_servoing = False
        self.docking_in_progress = False
        # Clear last detection time
        self.last_detection_time_ = None
        # Re-invoke start_docking to go to the staging pose again
        self.start_docking()

    def move_camera_down_45_and_rotate_180(self):
        msg = Float64MultiArray()
        # Example angles
        msg.data = [
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            -3.14159, -0.7854,
            0.0, 0.0
        ]
        self.joint_pose_pub.publish(msg)
        self.get_logger().info('Moved camera to look behind (down 45°, rotate 180°).')

def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    
    # Inmediately start the docking process
    node.start_docking()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()