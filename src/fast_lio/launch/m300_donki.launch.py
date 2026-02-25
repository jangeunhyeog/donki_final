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

    # 1. M300 LiDAR Driver (remap to /m300/pointcloud to avoid conflict with merger output)
    m300_driver_node = Node(
        package='pacecat_m300_driver',
        executable='driver',
        name='pacecat_m300_driver',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(m300_driver_pkg, 'params', 'LDS-M300-E.yaml')],
        remappings=[('pointcloud', '/m300/pointcloud')]
    )

    # 2. CameraSDK LiDAR Node (depth camera, 전방 120°×80°)
    camera_node = Node(
        package='lx_camera_ros',
        executable='lx_camera_node',
        namespace='lx_camera_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'enable_gpu': 0},
            {'ip': ''},
            {'log_level': 1},
            {'log_path': './log/'},
            {'is_depth': 1},
            {'is_xyz': 1},
            {'LX_BOOL_ENABLE_3D_AMP_STREAM': 0},
            {'LX_BOOL_ENABLE_2D_STREAM': 0},
            {'LX_BOOL_ENABLE_IMU': 0},
            {'LX_INT_XYZ_UNIT': 1},  # 1: meters
            # Camera pose relative to base_link (SDK 내부 TF: base_link → mrdvs_tof)
            {'x': 0.3209},
            {'y': -0.0080},
            {'z': 0.4023},
            {'roll': -90.0},
            {'pitch': 0.0},
            {'yaw': -90.0},
        ]
    )

    # 3. Point Cloud Merger (M300 + Camera → /pointcloud)
    merger_node = Node(
        package='lio_nav_bringup',
        executable='pointcloud_merger',
        name='pointcloud_merger',
        output='screen',
        parameters=[{
            'm300_topic': '/m300/pointcloud',
            'camera_topic': '/lx_camera_node/LxCamera_Cloud',
            'output_topic': '/pointcloud',
            'output_frame': 'base_scan',
            'sync_tolerance': 0.1,
        }]
    )

    # 4. Fast-LIO2 (Localization + Mapping) — subscribes to /pointcloud (merged)
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

    # 5. Elevation Mapping (카메라 지면 데이터 활용)
    # elevation_mapping_node = Node(
    #     package='fast_lio',
    #     executable='elevation_mapping_node',
    #     name='elevation_mapping_node',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         {'grid_resolution': 0.2},
    #         {'robot_height': 1.5},
    #         {'obstacle_height_thresh': 0.3},
    #         {'max_slope_degree': 45.0},
    #         {'min_valid_points': 5},
    #         {'sensor_height': 0.3},
    #         {'ceiling_cutoff': 1.5},
    #         {'local_map_size': 30.0},
    #         {'active_map_size': 30.0},
    #         {'input_topic': '/cloud_registered'},
    #         {'map_frame_id': 'camera_init'},
    #         {'robot_frame_id': 'body'},
    #         {'inflation_radius': 0.8},
    #         {'cost_scaling_factor': 5.0},
    #         {'robot_radius': 0.35}
    #     ]
    # )

    # 6. RViz (비활성화 — 원격 접속 시 Wayland 에러 방지)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', os.path.join(fast_lio_pkg, 'rviz', 'fastlio.rviz')],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )

    # 7. TF Links for Fast-LIO to Robot
    tf_odom_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init']
    )
    
    tf_body_basefootprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_body_basefootprint',
        arguments=['-0.3544', '0.0287', '-0.5848', '0', '0', '0', 'body', 'base_footprint']
    )

    # 8. Camera LiDAR TF — SDK가 base_link → mrdvs_tof TF를 자체 발행하므로
    #    별도 static TF 불필요 (SDK의 x/y/z/roll/pitch/yaw 파라미터로 설정)

    return LaunchDescription([
        declare_sim_time,
        m300_driver_node,
        camera_node,
        merger_node,
        fast_lio_node,
        #elevation_mapping_node,
        tf_odom_camera,
        tf_body_basefootprint,
    ])
