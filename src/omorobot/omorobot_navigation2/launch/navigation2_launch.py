import os
from launch_ros.actions import Node
from launch import LaunchDescription
from rclpy.logging import get_logger
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription

ROBOT_MODEL = os.getenv('ROBOT_MODEL', 'R2MINI')

def generate_launch_description():
    navigation_dir = get_package_share_directory('omorobot_navigation2')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    map = LaunchConfiguration('map', default=os.path.join(navigation_dir, 'map', 'turtlebot3_world.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(navigation_dir, 'param', ROBOT_MODEL+'.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam = LaunchConfiguration('slam', default='False')

    map_arg = DeclareLaunchArgument('map', default_value=map)
    params_file_arg = DeclareLaunchArgument('params_file', default_value=params_file)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    slam_arg = DeclareLaunchArgument('slam', default_value=slam)

    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map, 
            'params_file': params_file, 
            'use_sim_time': use_sim_time,
            'slam': slam,
            'use_robot_state_pub': 'False'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(map_arg)
    ld.add_action(params_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_arg)
    ld.add_action(navigation_node)

    return ld
