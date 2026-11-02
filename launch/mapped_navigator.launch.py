"""
Run mapped_navigator on top of a running navigation launch.

    ros2 launch stretch4_rtabmap mapped_navigator.launch.py

The navigation launch (navigation_mppi_glim_rtabmap.launch.py) has to be up first; its
rtabmap_supervisor publishes the /localization_status this node waits on.

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('stretch4_rtabmap')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'mapped_navigator.yaml'),
            description='mapped_navigator parameters.'),
        Node(
            package='stretch4_rtabmap',
            executable='mapped_navigator',
            name='mapped_navigator',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
