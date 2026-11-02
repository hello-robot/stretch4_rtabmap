r"""
Navigation with GLIM and RTAB-Map on the Jetson (README_GPU.md).

    ros2 launch stretch4_rtabmap navigation_jetson.launch.py mode:=localization \
        database_path:=$HOME/stretch_maps/<my_map>.db use_rviz:=true

Runs navigation_mppi_glim_rtabmap.launch.py here, with its arguments, without GLIM and RTAB-Map,
and starts those on the Jetson over ssh (jetson.launch.py). Ctrl-C stops both. Paths are the same
on both computers; the database is copied with `ros2 run stretch4_rtabmap jetson push-map <db>`.
"""

import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from stretch4_rtabmap.tools.jetson import JETSON

# Arguments forwarded to the Jetson when given.
JETSON_ARGS = ('mode', 'database_path', 'rtabmap_params_file',
               'require_loop_closure_before_mapping', 'mast_roi_ratios', 'initial_pose')

# RTAB-Map gets stereo_sync's JPEG copy of the stereo pair (~140 KB a message); the raw one
# (~14 MB) does not get through Zenoh to the Jetson.
STEREO_RGBD_TOPIC = '/rtabmap/stereo_rgbd_image'
STEREO_REMAPS = [
    SetRemap('/rgbd_image/compressed', STEREO_RGBD_TOPIC),
    SetRemap('/rgbd_image', f'{STEREO_RGBD_TOPIC}_raw'),
]


def jetson_command(host, router, given):
    """Return the ssh command that runs the Jetson half with the given {argument: value}."""
    # No router: run.sh uses the one in ~/stretch_gpu/env.
    args = ([f'nuc_router:={router}'] if router else []) + [f'{k}:={v}' for k, v in given.items()]
    # -tt: closing the ssh session stops run.sh.
    return ['ssh', '-tt', '-o', 'BatchMode=yes', '-o', 'ServerAliveInterval=5',
            '-o', 'ServerAliveCountMax=3', host,
            'stretch_gpu/run.sh ' + ' '.join(shlex.quote(a) for a in args)]


def launch_setup(context):
    navigation = GroupAction(STEREO_REMAPS + [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('stretch4_rtabmap'), 'launch',
            'navigation_mppi_glim_rtabmap.launch.py')),
        launch_arguments={'start_glim': 'false', 'start_rtabmap': 'false'}.items())])
    actions = [navigation]

    if LaunchConfiguration('start_jetson').perform(context) != 'true':
        actions.append(LogInfo(msg=(
            'start_jetson:=false: start GLIM and RTAB-Map on the Jetson yourself '
            '(~/stretch_gpu/run.sh with the same mode and database_path).')))
        return actions

    given = {k: context.launch_configurations[k] for k in JETSON_ARGS
             if k in context.launch_configurations}
    jetson = ExecuteProcess(
        cmd=jetson_command(LaunchConfiguration('jetson_host').perform(context),
                           LaunchConfiguration('nuc_router').perform(context), given),
        name='jetson',
        output='screen',
        emulate_tty=True,
    )
    actions += [
        jetson,
        RegisterEventHandler(OnProcessExit(target_action=jetson, on_exit=[LogInfo(msg=(
            'The Jetson side (GLIM, RTAB-Map) exited: no odometry or map until it is back. '
            'Check it with `ros2 run stretch4_rtabmap jetson check`.'))])),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_jetson', default_value='true', choices=['true', 'false'],
            description='Start the Jetson half over ssh.'),
        DeclareLaunchArgument(
            'jetson_host', default_value=JETSON,
            description='ssh destination of the Jetson (key login, no password).'),
        DeclareLaunchArgument(
            'nuc_router', default_value='',
            description="This computer's Zenoh router as the Jetson reaches it, e.g. "
                        'tcp/<nuc-ip>:7447. Default: the address `jetson deploy` found.'),
        OpaqueFunction(function=launch_setup),
    ])
