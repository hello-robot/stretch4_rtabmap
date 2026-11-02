"""
GLIM (CUDA odometry), RTAB-Map and a Zenoh router, in the container on the Jetson.

Started over ssh by navigation_jetson.launch.py, or by hand on the Jetson:

    ~/stretch_gpu/run.sh mode:=localization database_path:=~/stretch_maps/x.db
"""

import glob
import os
import re
import shutil

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from stretch4_rtabmap.sensors.rtabmap_roi import MAST_ROI_RATIOS


def glim_config_dir():
    """Return a GLIM config directory: stretch4_rtabmap's files with the CUDA odometry."""
    base = os.path.join(get_package_share_directory('stretch4_rtabmap'), 'config')
    out = os.path.expanduser('~/.ros/stretch4_rtabmap_jetson/glim_config')
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out)
    for path in sorted(glob.glob(os.path.join(base, '*.json'))):
        shutil.copy(path, out)
    config = os.path.join(out, 'config.json')
    with open(config) as f:
        text = f.read()
    with open(config, 'w') as f:
        f.write(text.replace('config_odometry_cpu.json', 'config_odometry_gpu.json'))
    # libmemory_monitor.so is a panel of GLIM's viewer, which is not built on the Jetson.
    ros_config = os.path.join(out, 'config_ros.json')
    with open(ros_config) as f:
        text = f.read()
    with open(ros_config, 'w') as f:
        f.write(re.sub(r'\s*"libmemory_monitor\.so",?', '', text))
    return out


def launch_setup(context):
    config_dir = glim_config_dir()
    mode = LaunchConfiguration('mode').perform(context)
    # No file given: the file for the mode, as in navigation_mppi_glim_rtabmap.launch.py.
    params_file = LaunchConfiguration('rtabmap_params_file').perform(context) or os.path.join(
        get_package_share_directory('stretch4_rtabmap'), 'config',
        f'rtabmap_params_{"localization" if mode == "localization" else "mapping"}.yaml')
    nuc_router = LaunchConfiguration('nuc_router').perform(context)

    actions = [
        # Time for RTAB-Map to save the database on shutdown.
        SetLaunchConfiguration('sigterm_timeout', '120'),
        SetLaunchConfiguration('sigkill_timeout', '30'),
    ]

    start_router = LaunchConfiguration('start_router').perform(context) == 'true'
    if start_router:
        if not nuc_router:
            raise RuntimeError('nuc_router is empty: run `ros2 run stretch4_rtabmap jetson '
                               'deploy` on the robot computer, or pass nuc_router:=tcp/<ip>:7447')
        # The binary itself, not `ros2 run`, so SIGINT stops it.
        actions.append(ExecuteProcess(
            cmd=[os.path.join(get_package_prefix('rmw_zenoh_cpp'), 'lib', 'rmw_zenoh_cpp',
                              'rmw_zenohd')],
            name='zenoh_router',
            output='screen',
            sigterm_timeout='5',
            additional_env={'ZENOH_CONFIG_OVERRIDE': f'connect/endpoints=["{nuc_router}"]'},
        ))

    glim = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_rosnode',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[{'config_path': config_dir}])

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('stretch4_rtabmap'), 'launch', 'rtabmap_glim.launch.py')),
        launch_arguments={
            'start_inputs': 'false',   # the cameras are on the robot computer
            'localization': 'true' if mode == 'localization' else 'false',
            'glim_config_path': config_dir,
            'database_path': LaunchConfiguration('database_path'),
            'rtabmap_params_file': params_file,
            'require_loop_closure_before_mapping':
                LaunchConfiguration('require_loop_closure_before_mapping'),
            'mast_roi_ratios': LaunchConfiguration('mast_roi_ratios'),
            'initial_pose': LaunchConfiguration('initial_pose'),
            'delete_db_on_start': 'true',  # rtabmap_glim ignores it in localization
            'rtabmap_viz': 'false',
            'rviz': 'false',
        }.items())

    # Start the nodes once the router is up.
    nodes = [glim, rtabmap]
    actions += [TimerAction(period=2.0, actions=nodes)] if start_router else nodes
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='slam', choices=['slam', 'localization'],
            description='As in navigation_mppi_glim_rtabmap.launch.py.'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db',
            description='Path inside the container; ~/stretch_maps there is the '
                        "Jetson's ~/stretch_gpu/home/stretch_maps."),
        DeclareLaunchArgument(
            'rtabmap_params_file', default_value='',
            description='As in navigation_mppi_glim_rtabmap.launch.py; empty: the file for the '
                        'mode.'),
        DeclareLaunchArgument(
            'require_loop_closure_before_mapping', default_value='true',
            choices=['true', 'false']),
        DeclareLaunchArgument('mast_roi_ratios', default_value=MAST_ROI_RATIOS),
        DeclareLaunchArgument('initial_pose', default_value=''),
        DeclareLaunchArgument(
            'nuc_router', default_value='',
            description="The robot computer's Zenoh router, tcp/<nuc-ip>:7447 (run.sh passes "
                        'the one `jetson deploy` found).'),
        DeclareLaunchArgument(
            'start_router', default_value='true', choices=['true', 'false'],
            description='Start the Jetson Zenoh router (false if one already runs here).'),
        OpaqueFunction(function=launch_setup),
    ])
