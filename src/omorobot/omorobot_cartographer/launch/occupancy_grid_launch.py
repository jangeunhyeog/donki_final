import os
from launch_ros.actions import Node
from launch import LaunchDescription
from rclpy.logging import get_logger
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription

def generate_launch_description():
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    resolution = LaunchConfiguration('resolution', default='0.05')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    publish_period_sec_arg = DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec)
    resolution_arg = DeclareLaunchArgument('resolution', default_value=resolution)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-publish_period_sec', publish_period_sec, '-resolution', resolution],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(publish_period_sec_arg)
    ld.add_action(resolution_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(cartographer_occupancy_grid_node)

    return ld
