from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('px4_vision_autonomy')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=config_file,
            description='Path to the config file'
        ),

        Node(
            package='px4_vision_autonomy',
            executable='camera_viewer',
            name='camera_viewer',
            output='screen'
        ),
        Node(
            package='px4_vision_autonomy',
            executable='aruco_detector',
            name='aruco_detector',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),
        Node(
            package='px4_vision_autonomy',
            executable='vision_controller',
            name='vision_controller',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),
        Node(
            package='px4_vision_autonomy',
            executable='mavsdk_bridge',
            name='mavsdk_bridge',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),
    ])
