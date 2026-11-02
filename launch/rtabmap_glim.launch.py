"""
RTAB-Map mapping or localization on GLIM odometry, with the lidar and head stereo pair.

GLIM publishes <glim_map> -> odom -> base_footprint, so RTAB-Map corrects above that chain and
publishes map -> <glim_map>. Every RTAB-Map parameter comes from rtabmap_params_file; this launch
only sets the ones tied to its own arguments (ROI, loop-closure gating, mode).
"""

import json
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetRemap
from stretch4_rtabmap.localization.rtabmap_params import load_params_file, render
from stretch4_rtabmap.sensors.rtabmap_roi import MAST_ROI_RATIOS, validate_roi_ratios

STEREO_NS = '/cameras_head/stereo'
MASKED_STEREO_NS = '/cameras_head/stereo_masked'
STEREO_RGBD_TOPIC = '/rtabmap/stereo_rgbd_image'

# Set by this launch from its arguments; rejected if they appear in the params file.
PARAMS_OWNED_BY_LAUNCH = {
    'Kp/RoiRatios': 'mast_roi_ratios',
    'Vis/RoiRatios': 'mast_roi_ratios',
    'Rtabmap/StartNewMapOnLoopClosure': 'require_loop_closure_before_mapping',
    'Mem/IncrementalMemory': 'localization',
}

PARAM_KEY_RE = re.compile(r'^[A-Za-z][A-Za-z0-9]*/[A-Za-z0-9]+$')


def _is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def _load_glim_json(path):
    """Load a GLIM config; they contain C-style comments, which json.loads rejects."""
    with open(path) as f:
        text = f.read()
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    text = re.sub(r'//.*?$', '', text, flags=re.MULTILINE)
    return json.loads(text)


def _resolve_odom_frame(context):
    """
    Return the frame RTAB-Map corrects against: GLIM's map frame, not odom.

    GLIM always publishes <map_frame_id> -> odom, so RTAB-Map publishing map -> odom would give
    odom two parents.
    """
    explicit = LaunchConfiguration('odom_frame_id').perform(context)
    if explicit:
        return explicit

    config_dir = LaunchConfiguration('glim_config_path').perform(context)
    index = _load_glim_json(os.path.join(config_dir, 'config.json'))
    ros_file = index.get('global', {}).get('config_ros', 'config_ros.json')
    ros_config = _load_glim_json(os.path.join(config_dir, ros_file))
    frame = ros_config.get('glim_ros', {}).get('map_frame_id', 'map')
    if frame == 'map':
        raise RuntimeError(
            f'GLIM map_frame_id is "map" in {config_dir}/{ros_file}, which collides with '
            "RTAB-Map's map frame. Set it to e.g. 'glim_map'.")
    return frame


def _params_file_args(context):
    """Return (rtabmap_args, set_parameter_actions, log_note) from rtabmap_params_file."""
    path = os.path.expanduser(LaunchConfiguration('rtabmap_params_file').perform(context))
    if not os.path.isfile(path):
        raise RuntimeError(f'rtabmap_params_file does not exist: "{path}"')

    params, chain = load_params_file(path)
    args, set_params = [], []
    for key, value in params.items():
        if key in PARAMS_OWNED_BY_LAUNCH:
            raise RuntimeError(
                f'{path}: {key} is set by the "{PARAMS_OWNED_BY_LAUNCH[key]}" launch argument; '
                'remove it from the file.')
        if not PARAM_KEY_RE.match(str(key)):
            raise RuntimeError(f'{path}: "{key}" is not a Group/Name RTAB-Map parameter.')
        if value is None:
            raise RuntimeError(f'{path}: {key} has no value.')
        rendered = render(value)
        # Arguments are whitespace-tokenized, so multi-token values go in as ROS parameters.
        if any(c.isspace() for c in rendered):
            set_params.append(SetParameter(key, rendered))
        else:
            args.append(f'--{key} {rendered}')

    note = LogInfo(msg=(
        f'rtabmap_glim: {len(params)} parameters from '
        + ' <- '.join(os.path.basename(p) for p in reversed(chain))))
    return args, set_params, note


def _launch_setup(context):
    localization = _is_true(context, 'localization')
    start_rtabmap = _is_true(context, 'start_rtabmap')
    start_inputs = _is_true(context, 'start_inputs')
    odom_frame_id = _resolve_odom_frame(context)

    rtabmap_args = []
    if _is_true(context, 'delete_db_on_start') and not localization:
        rtabmap_args.append('--delete_db_on_start')
    if _is_true(context, 'require_loop_closure_before_mapping'):
        rtabmap_args.append('--Rtabmap/StartNewMapOnLoopClosure true')

    file_args, file_set_params, file_note = _params_file_args(context)
    rtabmap_args.extend(file_args)

    roi = LaunchConfiguration('mast_roi_ratios').perform(context)
    validate_roi_ratios(roi)

    included_args = {
        # Stereo goes through stereo_sync + subscribe_rgbd: rtabmap_launch's own stereo path
        # cannot be combined with subscribe_scan_cloud and silently drops the lidar.
        'stereo': 'false',
        'depth': 'false',
        'subscribe_rgb': 'false',
        'subscribe_rgbd': 'true',
        'rgbd_topic': STEREO_RGBD_TOPIC,
        'subscribe_scan_cloud': 'true',
        'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
        'frame_id': LaunchConfiguration('frame_id'),
        'odom_frame_id': odom_frame_id,
        'map_frame_id': LaunchConfiguration('map_frame_id'),
        'localization': 'true' if localization else 'false',
        'map_topic': '/map',  # Nav2's static layer subscribes here
        'initial_pose': LaunchConfiguration('initial_pose'),
        'visual_odometry': 'false',
        'icp_odometry': 'false',
        'approx_sync': 'true',
        'wait_for_transform': LaunchConfiguration('wait_for_transform'),
        'qos': '2',
        'rviz': LaunchConfiguration('rviz'),
        'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
        'database_path': LaunchConfiguration('database_path'),
        'rtabmap_args': ' '.join(a for a in rtabmap_args if a),
    }

    # robot_self_mask paints the robot's own body out of both images (and throttles them), so
    # visual features and loop closures never match on the arm, whatever its configuration.
    mask_robot = _is_true(context, 'mask_robot')
    stereo_in = MASKED_STEREO_NS if mask_robot else STEREO_NS
    self_mask_nodes = []
    if mask_robot:
        self_mask_nodes.append(Node(
            package='stretch4_rtabmap',
            executable='robot_self_mask',
            name='robot_self_mask',
            output='log',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'max_rate': float(LaunchConfiguration('mask_max_rate').perform(context)),
                'padding': float(LaunchConfiguration('mask_padding').perform(context)),
                'fill_mode': LaunchConfiguration('mask_fill_mode').perform(context),
                'left_static_mask': LaunchConfiguration('mask_left_static').perform(context),
                'right_static_mask': LaunchConfiguration('mask_right_static').perform(context),
            }],
            remappings=[
                (f'{side}/{topic}_in', f'{STEREO_NS}/{side}/{topic}')
                for side in ('left', 'right') for topic in ('image_rect', 'camera_info')
            ] + [
                (f'{side}/{topic}_out', f'{MASKED_STEREO_NS}/{side}/{topic}')
                for side in ('left', 'right') for topic in ('image_rect', 'camera_info')
            ],
        ))

    stereo_sync = Node(
        package='rtabmap_sync',
        executable='stereo_sync',
        name='stereo_sync',
        output='log',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{'approx_sync': True, 'qos': 2, 'qos_camera_info': 2}],
        remappings=[
            ('left/image_rect', f'{stereo_in}/left/image_rect'),
            ('right/image_rect', f'{stereo_in}/right/image_rect'),
            ('left/camera_info', f'{stereo_in}/left/camera_info'),
            ('right/camera_info', f'{stereo_in}/right/camera_info'),
            ('rgbd_image', STEREO_RGBD_TOPIC),
        ],
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')),
        launch_arguments=included_args.items(),
    )

    scoped = []
    if _is_true(context, 'accept_initialpose'):
        # rtabmap listens on /rtabmap/initialpose; RViz publishes /initialpose.
        scoped.append(SetRemap('initialpose', '/initialpose'))
    # Always set, so a stale value stored in the database is overridden.
    scoped.append(SetParameter('Kp/RoiRatios', roi))
    scoped.append(SetParameter('Vis/RoiRatios', roi))
    scoped.extend(file_set_params)

    actions = []
    if start_inputs:
        # The stereo pair and its camera_stereo_*_optical_link TFs come from the luxonis camera
        # node, which only publishes them with use_stereo:=true.
        actions += self_mask_nodes + [stereo_sync]
    if start_rtabmap:
        actions += [
            LogInfo(msg=f'rtabmap_glim: publishing map -> {odom_frame_id}'),
            file_note,
            GroupAction(scoped + [rtabmap]),
        ]
    return actions


def generate_launch_description():
    share = get_package_share_directory('stretch4_rtabmap')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rtabmap_params_file',
            default_value=os.path.join(share, 'config', 'rtabmap_params_mapping.yaml'),
            description='RTAB-Map parameters under a "rtabmap_parameters:" key.'),
        DeclareLaunchArgument(
            'localization', default_value='false', choices=['true', 'false'],
            description='Localize against the database read-only instead of mapping.'),
        DeclareLaunchArgument(
            'require_loop_closure_before_mapping', default_value='false',
            choices=['true', 'false'],
            description='Add no nodes until a loop closure links into the existing map.'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),
        DeclareLaunchArgument(
            'delete_db_on_start', default_value='true', choices=['true', 'false'],
            description='Ignored in localization mode.'),
        DeclareLaunchArgument(
            'initial_pose', default_value='',
            description='Localization seed pose "x y z roll pitch yaw"; empty resumes the last '
                        'pose in the database.'),
        DeclareLaunchArgument(
            'accept_initialpose', default_value='true', choices=['true', 'false'],
            description='Accept /initialpose (RViz 2D Pose Estimate).'),
        DeclareLaunchArgument(
            'mast_roi_ratios', default_value=MAST_ROI_RATIOS,
            description='Image fractions "left right top bottom" cropped before feature '
                        'extraction.'),
        DeclareLaunchArgument(
            'mask_robot', default_value='true', choices=['true', 'false'],
            description="Paint the robot's own links out of the stereo images using the URDF "
                        'and TF, so features never match on the arm.'),
        DeclareLaunchArgument(
            'mask_max_rate', default_value='2.0',
            description='Max rate (Hz) of the masked images; RTAB-Map only uses 1 Hz.'),
        DeclareLaunchArgument(
            'mask_fill_mode', default_value='blur', choices=['constant', 'blur', 'inpaint'],
            description="How the robot's pixels are painted: constant (black), blur, or inpaint "
                        '(filled from the pixels around them).'),
        DeclareLaunchArgument(
            'mask_padding', default_value='0.03',
            description='Metres added to every side of each link box.'),
        DeclareLaunchArgument(
            'mask_left_static', default_value='',
            description='Optional PNG, non-zero pixels always masked in the left image.'),
        DeclareLaunchArgument(
            'mask_right_static', default_value='',
            description='Optional PNG, non-zero pixels always masked in the right image.'),
        DeclareLaunchArgument(
            'glim_config_path', default_value=os.path.join(share, 'config'),
            description='GLIM config directory, read for its map frame name.'),
        DeclareLaunchArgument(
            'odom_frame_id', default_value='',
            description="Frame RTAB-Map corrects against; empty uses GLIM's map frame."),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/lidar_points_both'),
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument(
            'wait_for_transform', default_value='0.5',
            description="Must exceed how far GLIM's TF lags the sensor stamps."),
        DeclareLaunchArgument(
            'start_rtabmap', default_value='true', choices=['true', 'false'],
            description='Start RTAB-Map. false when it runs on another computer '
                        '(navigation_jetson.launch.py).'),
        DeclareLaunchArgument(
            'start_inputs', default_value='true', choices=['true', 'false'],
            description="Start what feeds RTAB-Map's images: the stereo optical frames, "
                        'robot_self_mask and stereo_sync. They need the cameras, so they stay '
                        'on the robot computer when RTAB-Map runs elsewhere.'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_launch_setup),
    ])
