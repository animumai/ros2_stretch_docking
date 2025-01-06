from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=['config/docking_params.yaml'],
        ),
        Node(
            package='stretch_nav2_docking',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
        ),
    ])
