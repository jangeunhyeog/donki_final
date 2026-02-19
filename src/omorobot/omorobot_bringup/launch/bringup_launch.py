import os
from launch_ros.actions import Node
from launch import LaunchDescription
from rclpy.logging import get_logger
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition

LIDAR_MODEL = os.getenv('LIDAR_MODEL', 'TMINIPRO')
get_logger('bringup').info(f'LIDAR_MODEL: {LIDAR_MODEL}')
ROBOT_MODEL = os.getenv('ROBOT_MODEL', 'R2MINI')
get_logger('bringup').info(f'ROBOT_MODEL: {ROBOT_MODEL}')

def generate_launch_description():
    
    bringup_dir = get_package_share_directory('omorobot_bringup')
    description_dir = get_package_share_directory('omorobot_description')
    robot_dir = get_package_share_directory('omorobot_robot')

    if ROBOT_MODEL == 'R2MINI' or ROBOT_MODEL == 'DONKEYBOTI':
        lidar_yaml = LaunchConfiguration('lidar_yaml', default=os.path.join(bringup_dir, 'param', LIDAR_MODEL+'_'+ROBOT_MODEL+'.yaml'))
    else:
        lidar_yaml = LaunchConfiguration('lidar_yaml', default=os.path.join(bringup_dir, 'param', LIDAR_MODEL+'.yaml'))
    robot_yaml = LaunchConfiguration('robot_yaml', default=os.path.join(robot_dir, 'param', ROBOT_MODEL+'.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_pcl2scan = LaunchConfiguration('launch_pcl2scan', default='true')

    lidar_yaml_arg = DeclareLaunchArgument('lidar_yaml', default_value=lidar_yaml)
    robot_yaml_arg = DeclareLaunchArgument('robot_yaml', default_value=robot_yaml)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    launch_pcl2scan_arg = DeclareLaunchArgument('launch_pcl2scan', default_value=launch_pcl2scan, description='Launch pointcloud_to_laserscan node')

    robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_dir, '/launch/robot_launch.py']),
        launch_arguments={'robot_yaml': robot_yaml}.items()
    )

    # lidar_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([bringup_dir, '/launch/ydlidar_ros2_driver_launch.py']),
    #     launch_arguments={'lidar_yaml': lidar_yaml}.items()
    # )

    m300_pkg_dir = get_package_share_directory('pacecat_m300_driver')
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([m300_pkg_dir, '/launch/LDS-M300-E.launch']),
    )

    pointcloud_to_laserscan_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_dir, '/launch/pointcloud_to_laserscan_launch.py']),
        condition=IfCondition(launch_pcl2scan)
    )

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_dir, '/launch/robot_state_publisher_launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0.1005', '0', '0', '0', 'base_footprint', 'base_link']
    )

    ld = LaunchDescription()
    ld.add_action(lidar_yaml_arg)
    ld.add_action(robot_yaml_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(launch_pcl2scan_arg)
    ld.add_action(robot_node)
    ld.add_action(lidar_node)    
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(base_footprint_to_base_link)

    return ld
