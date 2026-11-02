r"""
Navigation on Stretch: GLIM odometry, RTAB-Map map and localization, Nav2 driving.

    ros2 launch stretch4_rtabmap navigation_mppi_glim_rtabmap.launch.py \
        mode:=localization database_path:=~/stretch_maps/<my_map>.db use_rviz:=true

Made of three parts, each in its own file:

    robot.launch.py         driver, cameras, lidars, IMU, GLIM odometry
    rtabmap_glim.launch.py  RTAB-Map and its image inputs (robot self-mask, stereo_sync)
    nav2.launch.py          Nav2, plus keepout/speed zones when keepout_mask/speed_mask are given

Arguments of those files can be given here too (e.g. mask_fill_mode:=constant for
rtabmap_glim.launch.py). This file adds rtabmap_supervisor (start_supervisor:=false to leave it
out): it tells whether the pose can be trusted (/localization_status, and as text in RViz),
cancels nav2 goals when it cannot, and in mode:=localization switches RTAB-Map to mapping once
localized (enable_mapping:=false keeps the map read-only). mapped_navigator:=true also starts
mapped_navigator and points RViz's goal tool and Navigation 2 panel at its navigate_mapped action.

TF ownership:

    map -> glim_map         RTAB-Map
    glim_map -> odom        GLIM (identity while enable_global_mapping is false)
    odom -> base_footprint  GLIM lidar-inertial odometry
    base_footprint -> ...   robot_state_publisher
"""

import os

from ament_index_python.packages import get_package_share_directory
from hello_helpers.launch_utils import _remove_qt_plugin_env_vars
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('stretch4_rtabmap')


def _launch(name, **arguments):
    """Include one of this package's launch files, keeping its launch configurations local."""
    return GroupAction([IncludeLaunchDescription(
        os.path.join(SHARE, 'launch', name), launch_arguments=arguments.items())])


def _is_true(context, name):
    raw = LaunchConfiguration(name).perform(context).strip().lower()
    if raw not in ('true', '1', 'false', '0'):
        raise RuntimeError(f'{name} must be true or false, got "{raw}".')
    return raw in ('true', '1')


def launch_setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    initial_pose = LaunchConfiguration('initial_pose').perform(context).strip()
    gate = _is_true(context, 'require_loop_closure_before_mapping')

    # No rtabmap_params_file given: use the file for the mode, and let the supervisor swap the
    # settings that differ between the mapping and localization files when it switches modes.
    # A file you pass is used as-is for the whole run; nothing is swapped.
    mode_files = {name: os.path.join(SHARE, 'config', f'rtabmap_params_{name}.yaml')
                  for name in ('mapping', 'localization')}
    own_params_file = LaunchConfiguration('rtabmap_params_file').perform(context)

    actions = [_launch(
        'rtabmap_glim.launch.py',
        localization='true' if mode == 'localization' else 'false',
        rtabmap_params_file=own_params_file or mode_files[
            'localization' if mode == 'localization' else 'mapping'],
        require_loop_closure_before_mapping='true' if gate else 'false',
        # robot.launch.py's argument; its own default when not given.
        glim_config_path=LaunchConfiguration(
            'config_path', default=os.path.join(SHARE, 'config')),
        database_path=LaunchConfiguration('database_path'),
        initial_pose=initial_pose,
        rtabmap_viz=LaunchConfiguration('rtabmap_viz'),
        rviz='false',
        delete_db_on_start='true',  # rtabmap_glim ignores it in localization
    )]

    if _is_true(context, 'start_supervisor'):
        actions.append(Node(
            package='stretch4_rtabmap',
            executable='rtabmap_supervisor',
            name='rtabmap_supervisor',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[LaunchConfiguration('supervisor_params_file'), {
                # slam: a new map has nothing to link to, so only report.
                'manage_mode': mode == 'localization',
                'enable_mapping': _is_true(context, 'enable_mapping'),
                # The supervisor turns the gate off in localization and back on for mapping.
                'gate_in_mapping': gate,
                # A seed pose given to RTAB-Map counts like a 2D Pose Estimate.
                'trust_start_pose': bool(initial_pose),
                'mapping_params_file': '' if own_params_file else mode_files['mapping'],
                'localization_params_file': '' if own_params_file else mode_files['localization'],
            }]))

    if mode == 'slam':
        actions.append(LogInfo(msg=(
            'mode:=slam ignores /initialpose and initial_pose; use mode:=localization to set '
            'the pose by hand.')))

    mapped = _is_true(context, 'mapped_navigator')
    if mapped:
        actions.append(_launch('mapped_navigator.launch.py'))

    if _is_true(context, 'use_rviz'):
        remappings = [(f'navigate_to_pose{suffix}', f'navigate_mapped{suffix}')
                      for suffix in ('', '/_action/status', '/_action/feedback')] if mapped else []
        # Restarted if it crashes, and never stops the rest.
        actions += _remove_qt_plugin_env_vars() + [Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            respawn=True,
            respawn_delay=5.0,
            # stretch_nav2's navigation.rviz plus the localization status text.
            arguments=['-d', os.path.join(SHARE, 'rviz', 'navigation_rtabmap.rviz')],
            remappings=remappings,
        )]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='slam', choices=['slam', 'localization'],
            description='slam: make a new map (deletes the database at database_path first). '
                        'localization: use an existing one (and keep '
                        'it up to date once localized, unless enable_mapping:=false).'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db',
            description='RTAB-Map database (the map).'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false', description='Open RViz. true/false.'),
        DeclareLaunchArgument(
            'initial_pose', default_value='',
            description='Localization seed pose "x y z roll pitch yaw"; empty resumes the last '
                        'pose in the database.'),
        DeclareLaunchArgument(
            'enable_mapping', default_value='true',
            description='mode:=localization only: let the supervisor switch RTAB-Map to mapping '
                        'once localized. false keeps the map read-only. true/false.'),
        DeclareLaunchArgument(
            'rtabmap_params_file', default_value='',
            description='RTAB-Map parameters under a "rtabmap_parameters:" key. Empty: '
                        'rtabmap_params_localization.yaml in mode:=localization (switched to '
                        'the rtabmap_params_mapping.yaml values while mapping), '
                        'rtabmap_params_mapping.yaml in mode:=slam.'),
        DeclareLaunchArgument(
            'require_loop_closure_before_mapping', default_value='true',
            choices=['true', 'false'],
            description='Add no nodes until a loop closure links into the existing map, so a '
                        'new session cannot start a disconnected map.'),
        DeclareLaunchArgument(
            'start_supervisor', default_value='true', choices=['true', 'false'],
            description='Start rtabmap_supervisor (localization status, mode switching).'),
        DeclareLaunchArgument(
            'supervisor_params_file',
            default_value=os.path.join(SHARE, 'config', 'rtabmap_supervisor.yaml'),
            description='rtabmap_supervisor parameters.'),
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='false', choices=['true', 'false'],
            description="RTAB-Map's own UI."),
        DeclareLaunchArgument(
            'mapped_navigator', default_value='false', choices=['true', 'false'],
            description='Start mapped_navigator and send RViz goals to its navigate_mapped '
                        "action instead of Nav2's navigate_to_pose."),

        # Every process (RTAB-Map saving a large database above all) gets time to shut down.
        SetLaunchConfiguration('sigterm_timeout', '120'),
        SetLaunchConfiguration('sigkill_timeout', '30'),
        _launch('robot.launch.py'),
        OpaqueFunction(function=launch_setup),
        _launch('nav2.launch.py'),
    ])
