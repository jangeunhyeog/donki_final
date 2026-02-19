import os
from launch_ros.actions import Node
from launch import LaunchDescription
from rclpy.logging import get_logger
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription

LIDAR_MODEL = os.getenv('LIDAR_MODEL', 'TMINIPRO')

def generate_launch_description():
    bringup_dir = get_package_share_directory('omorobot_bringup')

    lidar_yaml = LaunchConfiguration('lidar_yaml', default=os.path.join(bringup_dir, 'param', LIDAR_MODEL+'.yaml'))

    lidar_yaml_arg = DeclareLaunchArgument('lidar_yaml', default_value=lidar_yaml)

    lidar_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_yaml],
        namespace='/'
    )

    ld = LaunchDescription()
    ld.add_action(lidar_yaml_arg)
    ld.add_action(lidar_node)

    return ld
