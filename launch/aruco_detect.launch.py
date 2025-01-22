import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Example: provide parameter overrides
    marker_id = 245
    marker_size = 0.11  # 110mm

    return LaunchDescription([
        Node(
            package='ros2_stretch_docking',
            executable='aruco_detect_node',
            name='aruco_detect_node',
            output='screen',
            parameters=[{
                'marker_id': marker_id,
                'marker_size_meters': marker_size
            }]
        ),
    ])
