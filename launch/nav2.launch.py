r"""
Nav2 (MPPI) on RTAB-Map's /map, with optional keepout and speed zones.

Included by navigation_mppi_glim_rtabmap.launch.py; not meant to run alone.

Zones are nav2 costmap filters. Give a mask and its filter is turned on; leave it empty and nothing
about it is loaded:

    keepout_mask:=<keepout.yaml>   KeepoutFilter, global + local costmaps
    speed_mask:=<speed.yaml>       SpeedFilter, global costmap only (nav2: never on both)
"""

import os
import tempfile

from hello_helpers.multi_yaml import MultiYaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml


def _zones_overlay(keepout, speed):
    """Nav2 parameters that turn on exactly the filters with a mask."""
    common = {'enabled': True, 'transform_tolerance': 0.1}
    global_params, local_params = {'filters': []}, {}
    if keepout:
        keepout_filter = {'plugin': 'nav2_costmap_2d::KeepoutFilter',
                          'filter_info_topic': '/keepout_costmap_filter_info', **common}
        global_params['filters'].append('keepout_filter')
        global_params['keepout_filter'] = keepout_filter
        local_params = {'filters': ['keepout_filter'], 'keepout_filter': dict(keepout_filter)}
    if speed:
        global_params['filters'].append('speed_filter')
        global_params['speed_filter'] = {
            'plugin': 'nav2_costmap_2d::SpeedFilter',
            'filter_info_topic': '/speed_costmap_filter_info',
            'speed_limit_topic': '/speed_limit', **common}
    overlay = {'global_costmap': {'global_costmap': {'ros__parameters': global_params}}}
    if local_params:
        overlay['local_costmap'] = {'local_costmap': {'ros__parameters': local_params}}
    return overlay


def _mask(context, name):
    path = os.path.expanduser(LaunchConfiguration(name).perform(context).strip())
    if path and not os.path.isfile(path):
        raise RuntimeError(f'{name}: no such file "{path}"')
    return path


def launch_setup(context):
    nav2_path = FindPackageShare('stretch_nav2')
    nav2_config = PathJoinSubstitution([nav2_path, 'config'])

    # Later files deep-merge over earlier ones, so the GLIM overrides win.
    params = [
        PathJoinSubstitution([nav2_config, 'original_nav2_params.yaml']),
        PathJoinSubstitution([nav2_config, 'nav2_params_core.yaml']),
        PathJoinSubstitution([nav2_config, 'nav2_params_mppi.yaml']),
        PathJoinSubstitution([nav2_config, 'mppi_params.yaml']),
        PathJoinSubstitution([FindPackageShare('stretch4_rtabmap'), 'config',
                              'glim_overrides.yaml']),
    ]

    actions = []
    keepout, speed = _mask(context, 'keepout_mask'), _mask(context, 'speed_mask')
    if keepout or speed:
        overlay = tempfile.NamedTemporaryFile(mode='w', prefix='nav2_zones_', suffix='.yaml',
                                              delete=False)
        yaml.safe_dump(_zones_overlay(keepout, speed), overlay)
        overlay.close()
        params.append(overlay.name)
        # The mask servers start now; nav2 starts nav2_start_delay later and finds them up.
        actions += [
            LogInfo(msg='zones: ' + ', '.join(
                f'{n} {p}' for n, p in (('keepout', keepout), ('speed', speed)) if p)),
            IncludeLaunchDescription(
                PathJoinSubstitution([nav2_path, 'launch', 'include', 'nav2_filters.launch.py']),
                launch_arguments={
                    'enable_keepout': 'true' if keepout else 'false',
                    'enable_speed': 'true' if speed else 'false',
                    'keepout_mask': keepout,
                    'speed_mask': speed,
                }.items()),
        ]

    extra = os.path.expanduser(LaunchConfiguration('nav2_extra_params').perform(context).strip())
    if extra:
        if not os.path.isfile(extra):
            raise RuntimeError(f'nav2_extra_params: no such file "{extra}"')
        params.append(extra)

    composition = LaunchConfiguration('use_composition').perform(context).strip().lower()
    nav2 = GroupAction([
        # A crash takes down only the crashed node.
        SetLaunchConfiguration('use_respawn', 'True'),
        IncludeLaunchDescription(
            PathJoinSubstitution([nav2_path, 'launch', 'include', 'nav_core.launch.py']),
            launch_arguments={
                'params_file': MultiYaml(params),
                # navigation_mppi_glim_rtabmap starts RViz itself: nav2's rviz_launch.py shuts
                # the whole launch down when RViz exits.
                'use_rviz': 'false',
                'use_composition': 'True' if composition in ('true', '1') else 'False',
                # No AMCL and no map_server: RTAB-Map localizes and publishes /map.
                'use_localization': 'False',
            }.items()),
    ])

    # Keeps Nav2 out of the startup CPU spike so its lifecycle bonds don't time out.
    delay = float(LaunchConfiguration('nav2_start_delay').perform(context))
    actions.append(TimerAction(period=delay, actions=[nav2]) if delay > 0.0 else nav2)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'keepout_mask', default_value='',
            description='map_server yaml of a keepout mask (black = never enter). Empty: none.'),
        DeclareLaunchArgument(
            'speed_mask', default_value='',
            description='map_server yaml of a speed mask (mode: scale; grey = slower). '
                        'Empty: none.'),
        DeclareLaunchArgument(
            'nav2_extra_params', default_value='',
            description='Optional Nav2 params file merged last, over every other one.'),
        DeclareLaunchArgument(
            'nav2_start_delay', default_value='5.0',
            description='Seconds to wait before starting Nav2; 0 disables.'),
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Run Nav2 as composed components. true/false, either casing.'),
        OpaqueFunction(function=launch_setup),
    ])
