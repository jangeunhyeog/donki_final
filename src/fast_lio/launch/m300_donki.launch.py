import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    fast_lio_pkg = get_package_share_directory('fast_lio')
    m300_driver_pkg = get_package_share_directory('pacecat_m300_driver')
    lio_nav_pkg = get_package_share_directory('lio_nav_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock'
    )

    # 1. M300 LiDAR Driver
    m300_driver_node = Node(
        package='pacecat_m300_driver',
        executable='driver',
        name='pacecat_m300_driver',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(m300_driver_pkg, 'params', 'LDS-M300-E.yaml')]
    )

    # 2. Fast-LIO2 (Localization + Mapping)
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            os.path.join(fast_lio_pkg, 'config', 'm300_donki.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    # 3. Elevation Mapping (비활성화 — 현재 Task에서 미사용)
    # elevation_mapping_node = Node(
    #     package='fast_lio',
    #     executable='elevation_mapping_node',
    #     name='elevation_mapping_node',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         {'grid_resolution': 0.2},
    #         {'map_size_x': 100.0},
    #         {'map_size_y': 100.0},
    #         {'robot_height': 1.5},
    #         {'obstacle_height_thresh': 0.3},
    #         {'max_slope_degree': 45.0},
    #         {'min_valid_points': 5},
    #         {'sensor_height': 0.3},
    #         {'ceiling_cutoff': 1.5},
    #         {'input_topic': '/cloud_registered'},
    #         {'map_frame_id': 'camera_init'},
    #         {'inflation_radius': 0.8},
    #         {'cost_scaling_factor': 5.0},
    #         {'robot_radius': 0.35}
    #     ]
    # )

    # 4. A* Path Planner (비활성화 — Nav2로 대체)
    # path_planner_node = Node(
    #     package='lio_nav_bringup',
    #     executable='path_planner_node.py',
    #     name='path_planner_node',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         {'planning_frequency': 2.0},
    #         {'control_frequency': 20.0},
    #         {'plan_unknown': True},
    #         {'lookahead_dist': 0.8},
    #         {'max_speed': 0.5},
    #         {'goal_tolerance': 0.3},
    #         {'cost_weight': 2.0},
    #         {'enable_movement': False}
    #     ]
    # )

    # 5. RViz (비활성화 — 원격 접속 시 Wayland 에러 방지)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', os.path.join(fast_lio_pkg, 'rviz', 'fastlio.rviz')],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )

    # 6. TF Links for Fast-LIO to Robot
    tf_odom_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init']
    )
    
    tf_body_basefootprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.3544', '0.0287', '-0.5848', '0', '0', '0', 'body', 'base_footprint']
    )

    return LaunchDescription([
        declare_sim_time,
        m300_driver_node,
        fast_lio_node,
        tf_odom_camera,
        tf_body_basefootprint,
    ])
