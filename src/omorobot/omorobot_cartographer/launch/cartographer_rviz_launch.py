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
    cartographer_dir = get_package_share_directory('omorobot_cartographer')

    rviz_file = LaunchConfiguration('rviz_file', default=os.path.join(cartographer_dir, 'rviz', 'cartographer.rviz'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_file_arg = DeclareLaunchArgument('rviz_file', default_value=rviz_file)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(rviz_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz2_node)

    return ld
