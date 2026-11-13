"""Robot side of navigation: driver, head stereo pair, lidars, footprint, IMU, GLIM odometry."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stretch_core_launch = PathJoinSubstitution([FindPackageShare('stretch_core'), 'launch'])
    share = get_package_share_directory('stretch4_rtabmap')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path', default_value=os.path.join(share, 'config'),
            description='GLIM config directory (contains config.json).'),
        DeclareLaunchArgument(
            'start_glim', default_value='true', choices=['true', 'false'],
            description='Start GLIM here. false when it runs on the Jetson '
                        '(navigation_jetson.launch.py).'),
        DeclareLaunchArgument(
            'enable_lever_arm', default_value='true', choices=['true', 'false'],
            description='IMU lever-arm compensation in imu_to_base_frame.'),
        DeclareLaunchArgument(
            'use_center_camera', default_value='false', choices=['true', 'false'],
            description='Stream the head center camera too (nothing here uses it).'),

        IncludeLaunchDescription(
            PathJoinSubstitution([stretch_core_launch, 'stretch_driver.launch.py']),
            launch_arguments={'broadcast_odom_tf': 'False', 'mode': 'navigation'}.items()),
        # Only the head stereo pair is used (RTAB-Map).
        IncludeLaunchDescription(
            PathJoinSubstitution([stretch_core_launch, 'luxonis.launch.py']),
            launch_arguments={
                'use_rviz': 'false', 'fps': '10', 'use_stereo': 'true',
                'stereo_max_rate': '2.0',  # 2x RTAB-Map's 1 Hz detection rate
                'use_center': LaunchConfiguration('use_center_camera'),
            }.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([stretch_core_launch, 'dual_hesai.launch.py']),
            launch_arguments={
                'tool_preset': 'auto',
                'use_rviz': 'false',
                'lidar_pipeline': 'fused',
                'enable_sor_filter': 'true',
                'scan_range_max': '5.0',
            }.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([stretch_core_launch, 'robot_footprint.launch.py']),
            launch_arguments={'tool_preset': 'auto'}.items()),

        Node(
            package='stretch4_rtabmap',
            executable='imu_to_base_frame',
            name='imu_frame_transformer',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{'enable_lever_arm': LaunchConfiguration('enable_lever_arm')}]),
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim_rosnode',
            output='screen',
            # A restarted GLIM starts odometry again from where the robot stands; RTAB-Map
            # corrects map -> glim_map at its next localization.
            respawn=True,
            respawn_delay=3.0,
            parameters=[{'config_path': LaunchConfiguration('config_path')}],
            condition=IfCondition(LaunchConfiguration('start_glim'))),
    ])
