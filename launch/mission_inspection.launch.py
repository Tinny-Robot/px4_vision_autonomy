import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('px4_vision_autonomy')
    models_dir = os.path.join(pkg_share, 'models')
    # world_file = os.path.join(pkg_share, 'worlds', 'inspection.sdf')
    
    # PX4 Directory
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    
 
    env = os.environ.copy()
    
    # Add user's local gazebo models
    # user_gazebo_models = os.path.expanduser('/home/mrrobot/.gazebo/models') # for my CI
    user_gazebo_models = os.path.expanduser('~/.gazebo/models')

    if 'GAZEBO_MODEL_PATH' in env:
        env['GAZEBO_MODEL_PATH'] += ':' + models_dir + ':' + user_gazebo_models
    else:
        env['GAZEBO_MODEL_PATH'] = models_dir + ':' + user_gazebo_models
        
    # For GZ_SIM_RESOURCE_PATH, we prefer the package models.
    # We EXCLUDE user_gazebo_models to avoid conflicting with broken Classis models when using Fuel.
    if 'GZ_SIM_RESOURCE_PATH' in env:
        env['GZ_SIM_RESOURCE_PATH'] += ':' + models_dir
    else:
        env['GZ_SIM_RESOURCE_PATH'] = models_dir
        
    # Set the world file
    world_name = 'inspection'
    env['PX4_SITL_WORLD'] = world_name
    env['PX4_GZ_WORLD'] = world_name
    # env['PX4_GZ_WORLD'] = world_file  # Tried using full path, but sitl_gazebo requires just the name
    # env['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.dirname(world_file) # attempted resource path fix

    # PX4 Command
    # We use gz_x500_mono_cam_down for downward facing camera
    px4_cmd = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_mono_cam_down'],
        cwd=px4_dir,
        output='screen',
        additional_env=env,
        shell=True
    )

    # ROS GZ Bridge
    # Bridges the Gazebo camera topic to ROS2
    # The Gazebo topic is /world/inspection/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
    # We bridge it and remap it to /camera
    gz_topic = '/world/inspection/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image'
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[gz_topic + '@sensor_msgs/msg/Image@gz.msgs.Image'],
        remappings=[(gz_topic, '/camera')],
        output='screen'
    )

    return LaunchDescription([
        ros_gz_bridge,
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
            output='screen',
            parameters=[{'marker_size': 0.5, 'camera_topic': '/camera'}]
        ),
        Node(
            package='px4_vision_autonomy',
            executable='mission_commander',
            name='mission_commander',
            output='screen',
            parameters=[{'flip_x': True, 'flip_y': True}]
        ),
        LogInfo(msg="Starting PX4 SITL with Inspection World..."),
        LogInfo(msg="NOTE: Ensure your drone model has a downward facing camera for landing!"),
        px4_cmd
    ])
