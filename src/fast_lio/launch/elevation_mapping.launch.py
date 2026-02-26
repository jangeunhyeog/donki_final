import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock'
    )

    # Elevation Mapping (카메라 지면 데이터 활용)
    elevation_mapping_node = Node(
        package='fast_lio',
        executable='elevation_mapping_node',
        name='elevation_mapping_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'grid_resolution': 0.2},
            {'robot_height': 1.5},
            {'obstacle_height_thresh': 0.3},
            {'max_slope_degree': 45.0},
            {'min_valid_points': 5},
            {'sensor_height': 0.3},
            {'ceiling_cutoff': 1.5},
            {'local_map_size': 30.0},
            {'active_map_size': 30.0},
            {'input_topic': '/cloud_registered'},
            {'map_frame_id': 'camera_init'},
            {'robot_frame_id': 'body'},
            {'inflation_radius': 0.8},
            {'cost_scaling_factor': 5.0},
            {'footprint_x_max': 0.35},
            {'footprint_x_min': -1.1},
            {'footprint_y_max': 0.45},
            {'footprint_y_min': -0.45}
        ]
    )

    return LaunchDescription([
        declare_sim_time,
        elevation_mapping_node
    ])
