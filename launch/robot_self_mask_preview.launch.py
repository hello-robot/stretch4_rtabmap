"""
Tune robot_self_mask live, without RTAB-Map.

Runs the mask node on the head stereo pair and opens its debug image, left and right side by
side, which shows the GFTT corners RTAB-Map would extract (red = on or next to the mask, green =
elsewhere).

    ros2 launch stretch4_rtabmap robot_self_mask_preview.launch.py

Also starts the driver (TF, /robot_description, /joint_states) and the head cameras with the
stereo pair on. start_robot:=false when they already run.

In the debug image: yellow = mask outline, magenta x = feature the unmasked image has on the
robot (what the mask removes), red o = feature still on or next to the mask, green o = elsewhere.

Change the mask while it runs and the next frame shows the effect, e.g.
    ros2 param set /robot_self_mask fill_mode constant
    ros2 param set /robot_self_mask apply_mask false     # compare with no mask at all
or use `ros2 run rqt_reconfigure rqt_reconfigure`.
"""

import importlib.util
import os

from hello_helpers.launch_utils import _remove_qt_plugin_env_vars
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _rtabmap_glim_launch():
    """Import rtabmap_glim.launch.py from this directory for its constants."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rtabmap_glim.launch.py')
    spec = importlib.util.spec_from_file_location('rtabmap_glim_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _launch_setup(context):
    glim = _rtabmap_glim_launch()
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    topics = [(side, topic)
              for side in ('left', 'right') for topic in ('image_rect', 'camera_info')]
    mask = Node(
        package='stretch4_rtabmap',
        executable='robot_self_mask',
        name='robot_self_mask',
        output='screen',
        parameters=[{
            'max_rate': float(value('max_rate')),
            'fill_mode': value('fill_mode'),
            'apply_mask': value('apply_mask') == 'true',
            'debug': True,
            'padding': float(value('padding')),
            'left_static_mask': value('left_static_mask'),
            'right_static_mask': value('right_static_mask'),
        }],
        remappings=(
            [(f'{s}/{t}_in', f'{glim.STEREO_NS}/{s}/{t}') for s, t in topics]
            + [(f'{s}/{t}_out', f'{glim.MASKED_STEREO_NS}/{s}/{t}') for s, t in topics]),
    )
    # Left and right side by side (the node's ~/debug/stereo).
    viewers = [Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='self_mask_debug',
        arguments=['/robot_self_mask/debug/stereo'],
    )]
    # Something imported during launch (OpenCV's pip package) points Qt at plugins rqt cannot
    # load; the viewers then die at start, as RViz does.
    return [mask] + _remove_qt_plugin_env_vars() + viewers


def generate_launch_description():
    stretch_core_launch = PathJoinSubstitution([FindPackageShare('stretch_core'), 'launch'])
    # As in robot.launch.py; each include scoped so its arguments do not leak into the next.
    robot = GroupAction(condition=IfCondition(LaunchConfiguration('start_robot')), actions=[
        GroupAction([IncludeLaunchDescription(
            PathJoinSubstitution([stretch_core_launch, 'stretch_driver.launch.py']),
            launch_arguments={'mode': 'navigation'}.items())]),
        GroupAction([IncludeLaunchDescription(
            PathJoinSubstitution([stretch_core_launch, 'luxonis.launch.py']),
            launch_arguments={'use_rviz': 'false', 'fps': '10', 'use_stereo': 'true',
                              'use_center': 'false'}.items())]),
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_robot', default_value='true', choices=['true', 'false'],
            description='Start the driver and head cameras too. false if they already run.'),
        robot,
        DeclareLaunchArgument(
            'fill_mode', default_value='blur', choices=['constant', 'blur', 'inpaint']),
        DeclareLaunchArgument(
            'apply_mask', default_value='true', choices=['true', 'false'],
            description='false: show the images unmasked, with the features RTAB-Map would get.'),
        DeclareLaunchArgument(
            'max_rate', default_value='5.0',
            description='Higher than in rtabmap_glim (2 Hz) so the preview feels live.'),
        DeclareLaunchArgument('padding', default_value='0.03'),
        DeclareLaunchArgument('left_static_mask', default_value=''),
        DeclareLaunchArgument('right_static_mask', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
