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
    teleop_keyboard = ExecuteProcess(
        cmd=['ros2', 'run', 'omorobot_teleop', 'teleop_keyboard'],
        output='screen',
        prefix='gnome-terminal --'
    )

    ld = LaunchDescription()
    ld.add_action(teleop_keyboard)

    return ld
