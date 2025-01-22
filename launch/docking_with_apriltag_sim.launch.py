import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for ROS nodes in this launch script'),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='False',
            description='Whether to apply a namespace to the navigation stack'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Whether to use composed Nav2 bringup'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo Sim) clock if true'),
        DeclareLaunchArgument(
            'init_pose_x',
            default_value='0.0',
            description='Initial position X coordinate'),
        DeclareLaunchArgument(
            'init_pose_y',
            default_value='0.0',
            description='Initial position Y coordinate'),
        DeclareLaunchArgument(
            'init_pose_yaw',
            default_value='0.0',
            description='Initial yaw orientation'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_directory(
                    'ros2_simulator_stretch'),
                'maps', 'supermarket.yaml'
            ),
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'nav_params_file',
            default_value=os.path.join(
                get_package_share_directory(
                    'stretch_nav2_offloaded'),
                'config', 'nav2_params_sim.yaml'
            ),
            description='Full path to navigation param file to load'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='True',
            description='Launch RViz if set to True'),
        DeclareLaunchArgument(
            'image',
            default_value='/camera_rotated/color/image_raw',
            description='Image topic'),
        DeclareLaunchArgument(
            'camera_info',
            default_value='/camera_rotated/color/camera_info',
            description='Camera info topic'),
    ]
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_composition = LaunchConfiguration('use_composition')
    use_sim_time = LaunchConfiguration('use_sim_time')
    init_pose_x = LaunchConfiguration('init_pose_x', default=0.0)
    init_pose_y = LaunchConfiguration('init_pose_y', default=0.0)
    init_pose_yaw = LaunchConfiguration('init_pose_yaw', default=0.0)
    map_dir = LaunchConfiguration('map')
    nav_params_file = LaunchConfiguration('nav_params_file',)
    launch_rviz = LaunchConfiguration('launch_rviz')
    image = LaunchConfiguration('image')
    camera_info = LaunchConfiguration('camera_info')
    ros2_stretch_docking_launch_dir = os.path.join(
        get_package_share_directory('ros2_stretch_docking'), 'launch')

    param_substitutions = {
        'x': init_pose_x,
        'y': init_pose_y,
        'yaw': init_pose_yaw
    }

    configured_params = RewrittenYaml(
        source_file=nav_params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    nova_carter_dock_params_dir = os.path.join(
        get_package_share_directory('ros2_stretch_docking'), 'params')
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    params_file = os.path.join(
        nova_carter_dock_params_dir, 'stretch_docking.yaml')

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['docking_server']}],
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_launch_dir, '/bringup_launch.py']),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_composition': use_composition,
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
        }.items(),
    )

    dock_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ros2_stretch_docking_launch_dir, 'apriltag_dock_pose_publisher.launch.py')),
    )

    default_rviz_config_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz', 'nav2_default_view.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config_path],
        output='screen',
        condition=IfCondition(launch_rviz)
    )

    return launch.LaunchDescription(launch_args + [
        dock_detection_launch,
        nav2_bringup_launch,
        docking_server,
        lifecycle_manager,
        rviz_node
    ])
