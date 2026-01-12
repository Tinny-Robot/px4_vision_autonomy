from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('px4_vision_autonomy')
    
    # Arguments
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=os.path.expanduser('~/PX4-Autopilot'),
        description='Path to PX4-Autopilot directory'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='0',
        description='Run Gazebo headless'
    )

    # PX4 SITL + Gazebo Command
    # We use the standard make command. 
    # Note: This assumes dependencies are met and the build system is ready.
    px4_cmd = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_depth'],
        cwd=LaunchConfiguration('px4_dir'),
        output='screen',
        shell=True
    )

    # Include the perception stack
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'perception_only.launch.py')
        )
    )

    return LaunchDescription([
        px4_dir_arg,
        headless_arg,
        LogInfo(msg="Starting PX4 SITL and Gazebo..."),
        px4_cmd,
        LogInfo(msg="Starting Perception Stack..."),
        perception_launch
    ])
