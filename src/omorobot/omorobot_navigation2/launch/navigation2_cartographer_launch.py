import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get configuration directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    omorobot_navigation2_dir = get_package_share_directory('omorobot_navigation2')
    
    # Context
    robot_model = os.environ.get('ROBOT_MODEL', 'R2MINI')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(omorobot_navigation2_dir, 'param', robot_model + '.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Launch Navigation (Planner, Controller, BT, Recoveries)
    # This excludes AMCL and Map Server, assuming Cartographer provides the map and transforms.
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'  # Auto-start lifecycle nodes
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(navigation_launch_cmd)

    return ld
