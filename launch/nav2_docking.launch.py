# ros2_stretch_docking/launch/nav2_docking.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory for ros2_stretch_docking if you need YAML/config
    docking_pkg_share = get_package_share_directory('ros2_stretch_docking')
    
    # Example config file if you still want to reference it
    # nav2_params_file = os.path.join(docking_pkg_share, 'config', 'nav2_params_sim.yaml')

    # We do NOT launch Nav2 here since 'stretch_nav2_offloaded' does it for us.
    # We only start the ArUco detection node + docking node.

    return LaunchDescription([
        # (1) ArUco detection node
        Node(
            package='ros2_stretch_docking',
            executable='aruco_detect_node',
            name='aruco_detect_node',
            output='screen',
            parameters=[{
                'marker_id': 245,
                'marker_size_meters': 0.11
            }]
        ),

        # (2) Docking node
        Node(
            package='ros2_stretch_docking',
            executable='docking_node',
            name='docking_node',
            output='screen',
            parameters=[
                # Keep or change these as needed
                {'map_yaml_file': ''},
                {'dock_pose': [-2, 0.55, 0.0]},
                {'staging_offset': [-0.7, 0.0]},
                {'external_detection_offsets': [-0.4, 0.0, -0.16]},
            ]
        ),
    ])