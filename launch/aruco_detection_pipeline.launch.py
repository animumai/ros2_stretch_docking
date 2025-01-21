#!/usr/bin/env python3

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # These are the values passed in by the top-level launch file
    image_topic = LaunchConfiguration('image')
    camera_info_topic = LaunchConfiguration('camera_info')

    return launch.LaunchDescription([
        # Declare the same arguments here, with defaults (if desired)
        DeclareLaunchArgument(
            'image',
            default_value='/camera_rotated/color/image_raw',
            description='Image topic to subscribe to'
        ),
        DeclareLaunchArgument(
            'camera_info',
            default_value='/camera_rotated/color/camera_info',
            description='Camera info topic to subscribe to'
        ),

        Node(
            package='ros2_stretch_docking',
            executable='dock_pose_publisher',
            name='dock_pose_publisher',
            output='screen',
            parameters=[
                {'dictionary_id': '6x6_250'},
                {'marker_size': 0.11},
                {'use_first_detection': False},
                {'dock_tag_id': 245},
            ],
            # Here we remap the *hard-coded* topics in dock_pose_publisher.cpp
            # to the user-specified topics from the main launch file
            remappings=[
                ('/camera_rotated/color/image_raw', image_topic),
                ('/camera_rotated/color/camera_info', camera_info_topic),
            ]
        ),

        Node(
            package='ros2_stretch_docking',
            executable='rotate_camera_node',
            name='rotate_camera_node',
            output='screen'
        ),
    ])