"""
Drive to a goal on an RTAB-Map map, exploring toward it when nav2 has no path.

Goals: the `navigate_mapped` action (nav2_msgs/NavigateToPose).
State: /mapped_navigator/state. Localization is rtabmap_supervisor's job; this node only reads
/localization_status.

"""

from enum import Enum
import math
import signal
import threading
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PolygonStamped, PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav2_msgs.srv import ClearCostmapAroundRobot, ClearEntireCostmap, GetCost
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from simple_sm import MachineError, StateMachine, step, transition
from std_msgs.msg import String

from stretch4_rtabmap.localization.localization_policy import BLOCKED, Status
from stretch4_rtabmap.navigation.frontiers import find_frontiers
from stretch4_rtabmap.navigation.nav2_diagnose import (
    describe_error, error_names, INSCRIBED, map_cells_under, points_inside, scan_points,
    step_out_target, transform_points)
from stretch4_rtabmap.navigation.viewpoints import find_viewpoints
import tf2_ros


class Step(str, Enum):
    IDLE = 'IDLE'
    WAIT_LOCALIZATION = 'WAIT_LOCALIZATION'
    PLAN = 'PLAN'
    DRIVE = 'DRIVE'
    EXPLORE = 'EXPLORE'
    UNSTUCK = 'UNSTUCK'


# Steps that move the robot or are about to; a blocked status stops them.
MOVING_STEPS = [Step.PLAN, Step.DRIVE, Step.EXPLORE, Step.UNSTUCK]
GOAL_STEPS = MOVING_STEPS + [Step.WAIT_LOCALIZATION]
RUNNING, SUCCEEDED, FAILED = 'running', 'succeeded', 'failed'

# Planner errors that a map update can fix; anything else will not be fixed by exploring.
_R = ComputePathToPose.Result
MAP_ERRORS = {_R.NO_VALID_PATH, _R.GOAL_OCCUPIED, _R.GOAL_OUTSIDE_MAP}


def parse_status(text):
    """Status from a /localization_status message ("STATUS | detail | hint"), or None."""
    try:
        return Status(text.split('|', 1)[0].strip())
    except ValueError:
        return None


def yaw_of(q):
    """Yaw of a quaternion message."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ActionCall:
    """One goal on an action client, polled from the state machine instead of awaited."""

    def __init__(self, node, client, what, ack_timeout=5.0):
        self.node, self.client, self.what, self.ack_timeout = node, client, what, ack_timeout
        self.goal_future = self.result_future = self.handle = None
        self.result, self.status, self.sent_at = None, None, 0.0

    def start(self, goal):
        self.goal_future = self.result_future = self.handle = None
        self.result, self.status = None, None
        self.sent_at = time.monotonic()
        if not self.client.server_is_ready():
            self.node.get_logger().error(f'{self.what}: action server not available',
                                         throttle_duration_sec=5.0)
            return False
        self.goal_future = self.client.send_goal_async(goal)
        return True

    def poll(self):
        timed_out = time.monotonic() - self.sent_at > self.ack_timeout
        if self.goal_future is None:
            # Server not available: FAILED only after ack_timeout, so a retry does not spin.
            return FAILED if timed_out else RUNNING
        if self.result_future is None:
            if not self.goal_future.done():
                if not timed_out:
                    return RUNNING
                self.drop_pending()
                return FAILED
            self.handle = self.goal_future.result()
            if self.handle is None or not self.handle.accepted:
                return FAILED
            self.result_future = self.handle.get_result_async()
        if not self.result_future.done():
            return RUNNING
        response = self.result_future.result()
        self.status, self.result = response.status, response.result
        return SUCCEEDED if response.status == GoalStatus.STATUS_SUCCEEDED else FAILED

    def cancel(self):
        if self.handle is not None and self.result_future is not None \
                and not self.result_future.done():
            self.handle.cancel_goal_async()
        elif self.goal_future is not None and not self.goal_future.done():
            self.drop_pending()

    def drop_pending(self):
        """Give up on a goal the server has not answered: cancel it if it is accepted later."""
        def cancel_if_accepted(future):
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
        self.goal_future.add_done_callback(cancel_if_accepted)
        self.goal_future = None


class DirectMove:
    """
    One straight move in the robot frame with the heading kept. Used to step out of an obstacle.

    `direction` is a (dx, dy) unit vector in the robot frame. Velocities go to the collision
    monitor's input; travel is measured on odom. The move ends when the distance is covered, when
    it makes no progress for 1.5 s (the collision monitor stopped it: blocked), on timeout, or
    when node.motion_safe() says the collision monitor cannot be trusted right now.
    """

    def __init__(self, node, direction, distance, speed):
        self.node, self.direction, self.distance, self.speed = node, direction, distance, speed
        self.start = self.started = self.progress_at = None
        self.progress_d = 0.0
        self.done = False

    def poll(self):
        return self.done

    def update(self):
        """Publish the velocity for this instant; called at the motion rate (20 Hz)."""
        if self.done:
            return
        now = time.monotonic()
        here = self.node.odom_pose()
        if here is None:
            self.finish('no odometry')
            return
        if self.start is None:
            self.start, self.started = here, now
            self.progress_at, self.progress_d = now, 0.0
            return
        travelled = math.hypot(here[0] - self.start[0], here[1] - self.start[1])
        unsafe = self.node.motion_safe()
        if unsafe:
            self.finish(unsafe)
        elif travelled >= self.distance - 0.01:
            self.finish('done')
        elif now - self.started > self.distance / self.speed * 2.0 + 2.0:
            self.finish('timeout')
        elif now - self.progress_at > 1.5:
            if travelled - self.progress_d < 0.02:
                self.finish('blocked')
            else:
                self.progress_at, self.progress_d = now, travelled
        else:
            twist = Twist()
            twist.linear.x = self.direction[0] * self.speed
            twist.linear.y = self.direction[1] * self.speed
            self.node.cmd_pub.publish(twist)

    def finish(self, why):
        self.stop()
        if why not in ('done', 'blocked', 'timeout'):
            self.node.get_logger().warn(f'step stopped: {why}')
        self.done = True

    def stop(self):
        self.node.cmd_pub.publish(Twist())


class MappedNavigator(Node):

    def __init__(self):
        super().__init__('mapped_navigator')
        p = self.declare_parameter
        p('map_frame', 'map')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('status_topic', '/localization_status')   # rtabmap_supervisor's
        p('wait_abort', 120.0)              # s in WAIT_LOCALIZATION before the goal is aborted
        p('max_drive_attempts', 2)          # nav2 failures with a valid plan before aborting
        p('plan_retries', 2)                # planner errors a map update cannot fix, retried
        p('clear_entirely_service', '/global_costmap/clear_entirely_global_costmap')
        p('clear_wait', 2.0)                # s after clearing; the global costmap updates at 1 Hz
        p('cmd_vel_topic', 'cmd_vel_nav')   # the collision monitor's cmd_vel_in_topic
        p('scan_topic', '/scan_filtered')   # the collision monitor's source
        p('footprint_topic', '/local_costmap/published_footprint')  # its approach polygon
        p('max_scan_age', 0.5)              # s; the collision monitor ignores scans past 1.0
        p('step_speed', 0.1)                # m/s of a step-out
        p('get_cost_service', '/global_costmap/get_cost_global_costmap')
        p('clear_around_service', '/global_costmap/clear_around_global_costmap')
        p('scan_hits_blocked', 3)           # scan points inside the footprint = really blocked
        p('clear_around_distance', 1.5)     # m; window cleared around the robot
        p('max_unstuck_tries', 2)           # step-outs per goal before aborting
        p('unstuck_distance', 0.5)          # m; farthest a step-out goes
        p('explore_radius', 4.0)            # m around the goal for frontiers and viewpoints
        p('robot_clearance', 0.25)          # m from obstacles; nav2 robot_radius is 0.19
        p('min_frontier_size', 0.5)         # m; shorter frontiers are ignored
        p('potential_scale', 1.0)           # frontier cost per m from the robot
        p('gain_scale', 1.0)                # frontier cost per m of frontier length (subtracted)
        p('goal_scale', 2.0)                # frontier cost per m from the goal
        p('viewpoint_spacing', 0.75)
        p('max_targets', 5)                 # targets visited per goal before aborting
        p('max_target_checks', 10)          # planner calls per target pick
        p('look_around', 'spin')            # at a target: 'spin' or 'none'
        p('settle_time', 3.0)               # s after looking around for /map to catch up
        self.cfg = {name: param.value
                    for name, param in self.get_parameters_by_prefix('').items()}
        if self.cfg['look_around'] not in ('spin', 'none'):
            raise ValueError("look_around must be 'spin' or 'none'")

        fsm = MutuallyExclusiveCallbackGroup()
        clients = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(String, self.cfg['status_topic'], self.on_status, latched,
                                 callback_group=fsm)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched, callback_group=fsm)
        self.state_pub = self.create_publisher(String, '~/state', latched)
        self.cmd_pub = self.create_publisher(Twist, self.cfg['cmd_vel_topic'], 10)
        self.scan_stamp = None      # header stamp of the latest scan
        self.scan = None            # latest scan, for the points inside the footprint
        self.footprint = None       # (K, 2) costmap footprint in the base frame, arm included
        self.footprint_at = None    # monotonic time the latest footprint arrived
        self.create_subscription(LaserScan, self.cfg['scan_topic'], self.on_scan,
                                 qos_profile_sensor_data, callback_group=fsm)
        self.create_subscription(PolygonStamped, self.cfg['footprint_topic'], self.on_footprint,
                                 10, callback_group=fsm)

        self.nav = ActionCall(self, ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                                 callback_group=clients), 'navigate_to_pose')
        self.nav_goals = []         # latest goal status list of nav2's navigate_to_pose
        self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status',
                                 self.on_nav_goals, 10, callback_group=fsm)
        self.plan = ActionCall(self, ActionClient(self, ComputePathToPose, 'compute_path_to_pose',
                                                  callback_group=clients), 'compute_path_to_pose')
        self.spin = ActionCall(self, ActionClient(self, Spin, 'spin', callback_group=clients),
                               'spin')
        self.get_cost = self.create_client(GetCost, self.cfg['get_cost_service'],
                                           callback_group=clients)
        self.clear_around = self.create_client(
            ClearCostmapAroundRobot, self.cfg['clear_around_service'], callback_group=clients)
        self.clear_entirely = self.create_client(
            ClearEntireCostmap, self.cfg['clear_entirely_service'], callback_group=clients)
        self.error_names = error_names()

        self.server = ActionServer(
            self, NavigateToPose, 'navigate_mapped', execute_callback=self.execute_goal,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=clients)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.status = None          # supervisor Status, None until one arrives
        self.status_text = 'no status yet'
        self.map = None
        self.map_count = 0
        self.goal = None            # PoseStamped being pursued
        self.goal_done = None       # Future resolved with (outcome, message, code)
        self.goal_handle = None
        self.pending = None         # (pose, done, handle) handed over by the action thread
        self.pending_lock = threading.Lock()
        self.entered = 0.0
        self.why = ''
        self.drive_failures = 0     # for the current goal
        self.plan_failures = 0      # planner errors exploring cannot fix, for the current goal
        self.costmap_cleared = False    # global costmap cleared once for the current goal
        self.clearing = None        # (future, replan_at) while PLAN waits for that clear
        self.unstuck = None         # UNSTUCK progress: phase, futures, tries
        self.unstuck_tries = 0      # step-outs for the current goal
        self.goal_code = 0          # nav2 error code of the latest failure, for the goal result
        self.visited = 0            # targets driven to for the current goal
        self.tried = []             # targets planned or visited for the current goal
        self.candidates = []
        self.target = None          # (x, y, yaw, label)
        self.phase = None           # EXPLORE phase: pick, go, look, settle
        self.phase_at = 0.0         # monotonic time the settle phase started
        self.moves = None           # DirectMove of a step-out
        self.map_count_at = 0

        self.machine = StateMachine(Step, initial=Step.IDLE, model=self)
        self.entered_state('waiting for a goal')
        self.create_timer(0.2, self.tick, callback_group=fsm)
        self.create_timer(0.05, self.motion_update, callback_group=fsm)

    # ---- transitions --------------------------------------------------------------------------
    # The bodies are empty: what a transition does is in the step handler that fires it, so each
    # handler reads top to bottom. The decorators are the transition table simple_sm enforces and
    # draws.

    @transition(source=Step.IDLE, dest=Step.PLAN)
    def goal_received(self):
        pass

    @transition(source=MOVING_STEPS, dest=Step.WAIT_LOCALIZATION)
    def blocked(self):
        pass

    @transition(source=Step.WAIT_LOCALIZATION, dest=Step.PLAN)
    def unblocked(self):
        pass

    @transition(source=Step.PLAN, dest=Step.DRIVE)
    def path_found(self):
        pass

    @transition(source=Step.PLAN, dest=Step.EXPLORE)
    def no_path(self):
        pass

    @transition(source=Step.DRIVE, dest=Step.IDLE)
    def arrived(self):
        pass

    @transition(source=Step.DRIVE, dest=Step.PLAN)
    def drive_failed(self):
        pass

    @transition(source=Step.EXPLORE, dest=Step.PLAN)
    def replan(self):
        pass

    @transition(source=Step.PLAN, dest=Step.UNSTUCK)
    def stuck(self):
        pass

    @transition(source=Step.UNSTUCK, dest=Step.PLAN)
    def unstuck_done(self):
        pass

    @transition(source=GOAL_STEPS, dest=Step.IDLE)
    def goal_ended(self):
        pass

    def fire(self, trigger, why):
        try:
            getattr(self, trigger)()
        except MachineError as error:
            # A transition missing from the table is a bug here, not a reason to keep driving.
            self.finish_goal('aborted', f'BUG, {error}')
            self.machine.set_state(Step.IDLE)
            why = f'BUG, {error}; goal aborted ({why})'
            self.get_logger().error(why)
        self.entered_state(why)

    def entered_state(self, why):
        self.entered, self.why = time.monotonic(), why
        state = self.machine.state
        self.get_logger().info(f'==== {state.value} ==== {why}')
        self.publish_state()

    def publish_state(self):
        status = self.status.value if self.status else 'unknown'
        self.state_pub.publish(String(data=f'{self.machine.state.value} | {self.why} | '
                                           f'localization {status}'))

    # ---- inputs -------------------------------------------------------------------------------

    def on_status(self, msg):
        status = parse_status(msg.data)
        if status is None:
            self.get_logger().error(f'unknown status on {self.cfg["status_topic"]}: {msg.data}',
                                    throttle_duration_sec=10.0)
        self.status_text = msg.data
        if status != self.status:
            self.status = status
            self.publish_state()

    def drivable(self):
        """Return True when the supervisor lets the robot drive."""
        return self.status is not None and self.status not in BLOCKED

    def on_scan(self, msg):
        self.scan_stamp = Time.from_msg(msg.header.stamp)
        self.scan = msg

    def on_footprint(self, msg):
        """Keep the costmap's footprint (robot_footprint_publisher's, arm included) in base."""
        self.footprint_at = time.monotonic()
        if len(msg.polygon.points) < 3:
            return
        try:
            t = self.tf_buffer.lookup_transform(self.cfg['base_frame'], msg.header.frame_id,
                                                Time()).transform
        except tf2_ros.TransformException:
            return
        yaw = yaw_of(t.rotation)
        self.footprint = transform_points([(p.x, p.y) for p in msg.polygon.points],
                                          (t.translation.x, t.translation.y, yaw))

    def scan_problem(self):
        """'' when the latest scan is fresh, else why not."""
        if self.scan_stamp is None:
            return f'no scan on {self.cfg["scan_topic"]}'
        age = (self.get_clock().now() - self.scan_stamp).nanoseconds * 1e-9
        if age > self.cfg['max_scan_age']:
            return f'scan is {age:.1f} s old; the collision monitor would ignore it'
        return ''

    def motion_safe(self):
        """'' when the collision monitor can stop a direct move, else why it cannot."""
        problem = self.scan_problem()
        if problem:
            return problem
        if self.footprint_at is None or time.monotonic() - self.footprint_at > 2.0:
            return 'no footprint from the local costmap (nav2 paused?)'
        return ''

    def motion_update(self):
        if self.moves is not None:
            self.moves.update()

    def on_map(self, msg):
        self.map = msg
        self.map_count += 1

    def on_nav_goals(self, msg):
        self.nav_goals = msg.status_list

    def nav_taken_over(self):
        """Return whether another client's goal is running on nav2, which then drops ours."""
        ours = bytes(self.nav.handle.goal_id.uuid) if self.nav.handle else None
        return any(bytes(g.goal_info.goal_id.uuid) != ours and g.status in (
            GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING) for g in self.nav_goals)

    async def execute_goal(self, handle):
        done = Future()
        # Runs on the action server's thread: hand the goal to tick() instead of touching the
        # state machine from here.
        with self.pending_lock:
            if self.pending is not None:
                self.pending[1].set_result(('aborted', 'replaced by a new goal', 0))
            self.pending = (handle.request.pose, done, handle)
        outcome, message, code = await done
        self.get_logger().info(f'goal {outcome}: {message}')
        if outcome == 'succeeded':
            handle.succeed()
        elif outcome == 'canceled':
            handle.canceled()
        else:
            handle.abort()
        result = NavigateToPose.Result()
        if hasattr(result, 'error_msg'):
            result.error_msg = message
        if hasattr(result, 'error_code') and outcome != 'succeeded':
            result.error_code = code
        return result

    def new_goal(self, pose, done, handle):
        if self.goal is not None:
            self.finish_goal('aborted', 'replaced by a new goal')
        if not pose.header.frame_id:
            pose.header.frame_id = self.cfg['map_frame']
        self.goal, self.goal_done, self.goal_handle = pose, done, handle
        self.tried, self.visited, self.drive_failures, self.plan_failures = [], 0, 0, 0
        self.unstuck_tries, self.goal_code = 0, 0
        self.costmap_cleared, self.clearing = False, None
        p = pose.pose.position
        self.get_logger().info(f'new goal ({p.x:.2f}, {p.y:.2f})')
        if self.machine.state in GOAL_STEPS:
            self.fire('goal_ended', 'goal replaced')

    def finish_goal(self, outcome, message):
        self.cancel_motion()
        if self.goal_done is not None and not self.goal_done.done():
            self.goal_done.set_result((outcome, message, self.goal_code))
        elif self.goal is not None:
            self.get_logger().info(f'goal {outcome}: {message}')
        self.goal = self.goal_done = self.goal_handle = None

    # ---- tick ---------------------------------------------------------------------------------

    def tick(self):
        with self.pending_lock:
            pending, self.pending = self.pending, None
        if pending is not None:
            self.new_goal(*pending)
        state = self.machine.state
        if self.goal_handle is not None and self.goal_handle.is_cancel_requested:
            self.finish_goal('canceled', 'canceled by the client')
            if state in GOAL_STEPS:
                self.fire('goal_ended', 'goal canceled')
            return
        if state in MOVING_STEPS and not self.drivable():
            # The supervisor cancels nav2's goals itself; this stops our own (spin, step-out)
            # and keeps the goal for when the pose can be trusted again.
            self.cancel_motion()
            self.fire('blocked', f'localization: {self.status_text}'
                      + (', goal kept' if self.goal else ''))
            return
        self.machine.step(self.status)

    def elapsed(self):
        return time.monotonic() - self.entered

    # ---- goals ----------------------------------------------------------------------------------

    @step(Step.IDLE)
    def on_idle(self, status):
        if self.goal is not None:
            self.fire('goal_received', 'goal pending')

    @step(Step.WAIT_LOCALIZATION)
    def on_wait_localization(self, status):
        if self.goal is None:
            self.fire('goal_ended', 'no goal')
        elif self.drivable():
            self.fire('unblocked', f'localization {status.value}; planning again')
        elif self.elapsed() > self.cfg['wait_abort']:
            reason = ('no /localization_status: is rtabmap_supervisor running?'
                      if self.status is None else f'localization: {self.status_text}')
            self.finish_goal('aborted', f'waited {self.elapsed():.0f} s; {reason}')
            self.fire('goal_ended', 'goal aborted')

    @step(Step.PLAN)
    def on_plan(self, status):
        if self.goal is None:
            self.fire('goal_ended', 'no goal')
            return
        if self.clearing is not None:
            future, replan_at = self.clearing
            if not future.done() or time.monotonic() < replan_at:
                return
            self.clearing = None
            self.plan.start(self.plan_goal(self.goal))
            return
        if self.plan.sent_at < self.entered:
            self.plan.start(self.plan_goal(self.goal))
            return
        result = self.plan.poll()
        if result == RUNNING:
            return
        if result == SUCCEEDED and self.plan.result.path.poses:
            self.plan_failures = 0
            self.nav.start(self.nav_goal(self.goal))
            self.fire('path_found', f'path of {len(self.plan.result.path.poses)} poses')
            return
        reason = self.plan_error()
        code = getattr(self.plan.result, 'error_code', None) if self.plan.result else None
        self.goal_code = code or 0
        if result != SUCCEEDED and code == _R.START_OCCUPIED:
            if self.unstuck_tries >= self.cfg['max_unstuck_tries']:
                self.finish_goal('aborted', f'{reason}; still in an obstacle after '
                                            f'{self.unstuck_tries} step-outs')
                self.fire('goal_ended', 'goal aborted')
            else:
                self.unstuck = {'phase': 'ask', 'future': None, 'cleared': False,
                                'wait_until': 0.0}
                self.fire('stuck', f'{reason}; checking what is really under the robot')
            return
        if result != SUCCEEDED and code not in MAP_ERRORS:
            # TF, timeout, planner down: a map update will not fix these.
            self.plan_failures += 1
            if self.plan_failures > self.cfg['plan_retries']:
                self.finish_goal('aborted', f'{reason}; not a map problem, so not exploring')
                self.fire('goal_ended', 'goal aborted')
            else:
                self.get_logger().warn(f'{reason}; retrying ({self.plan_failures})')
                self.plan.start(self.plan_goal(self.goal))
            return
        if not self.costmap_cleared:
            # Old scan marks in the global costmap are the cheapest explanation: clear them once
            # for this goal and plan again before exploring.
            self.costmap_cleared = True
            if self.clear_entirely.service_is_ready():
                self.get_logger().info(f'{reason}; clearing the global costmap and planning again')
                self.clearing = (self.clear_entirely.call_async(ClearEntireCostmap.Request()),
                                 time.monotonic() + self.cfg['clear_wait'])
                return
            self.get_logger().warn(f'{reason}; cannot clear the global costmap '
                                   f'({self.cfg["clear_entirely_service"]} not available)')
        if status != Status.MAPPING:
            self.finish_goal('aborted', f'{reason}; localization is {status.value}, not '
                                        'MAPPING, so the map cannot change')
            self.fire('goal_ended', 'goal aborted')
        elif self.visited >= self.cfg['max_targets']:
            self.finish_goal('aborted', f'{reason} after {self.visited} targets')
            self.fire('goal_ended', 'goal aborted')
        else:
            self.candidates = self.pick_candidates()
            self.phase, self.target = 'pick', None
            frontiers = sum(1 for c in self.candidates if c[3].startswith('frontier'))
            self.fire('no_path', f'{reason}; {frontiers} frontiers and '
                                 f'{len(self.candidates) - frontiers} viewpoints to check')

    @step(Step.DRIVE)
    def on_drive(self, status):
        result = self.nav.poll()
        if result == RUNNING:
            return
        if result == SUCCEEDED:
            self.finish_goal('succeeded', 'arrived')
            self.fire('arrived', 'arrived')
            return
        if self.nav_taken_over():
            # nav2 aborts a running goal when another client sends one (RViz, a script):
            # sending ours again would throw theirs out.
            self.finish_goal('aborted', 'replaced by another nav2 goal')
            self.fire('goal_ended', 'goal aborted')
            return
        if self.nav.status == GoalStatus.STATUS_CANCELED:
            # Not ours (we only cancel when leaving DRIVE): the supervisor, or another nav2
            # client. Plan again without counting it; PLAN waits if localization is blocked.
            self.fire('drive_failed', 'nav2 goal cancelled by someone else; replanning')
            return
        self.drive_failures += 1
        reason = self.nav_error()
        if self.drive_failures >= self.cfg['max_drive_attempts']:
            self.finish_goal('aborted', f'nav2 failed {self.drive_failures} times with a valid '
                                        f'plan; last: {reason}')
            self.fire('goal_ended', 'goal aborted')
        else:
            self.fire('drive_failed', f'nav2 failed: {reason}; replanning')

    @step(Step.EXPLORE)
    def on_explore(self, status):
        if self.phase == 'pick':
            self.explore_pick()
        elif self.phase == 'go':
            if self.nav.poll() == RUNNING:
                return
            if self.nav.status != GoalStatus.STATUS_SUCCEEDED and self.nav_taken_over():
                self.finish_goal('aborted', 'replaced by another nav2 goal')
                self.fire('goal_ended', 'goal aborted')
                return
            # Even a failed approach usually got closer and saw something; look around either
            # way.
            reached = self.nav.status == GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info('at the target' if reached
                                   else 'could not reach the target; looking from here')
            if self.cfg['look_around'] == 'spin':
                self.start_spin()
            self.phase = 'look'
        elif self.phase == 'look':
            if self.cfg['look_around'] == 'spin' and self.spin.poll() == RUNNING:
                return
            self.map_count_at, self.phase = self.map_count, 'settle'
            self.phase_at = time.monotonic()
        elif self.phase == 'settle':
            waited = time.monotonic() - self.phase_at
            if self.map_count > self.map_count_at and waited > 1.0 \
                    or waited > self.cfg['settle_time']:
                self.fire('replan', 'replanning to the goal on the updated map')

    def explore_pick(self):
        """Plan to the candidates one at a time; drive to the first one with a path."""
        if self.target is not None:
            result = self.plan.poll()
            if result == RUNNING:
                return
            x, y, yaw, label = self.target
            self.tried.append((x, y))
            self.target = None
            if result == SUCCEEDED and self.plan.result.path.poses:
                self.visited += 1
                self.nav.start(self.nav_goal(self.pose(x, y, yaw)))
                self.phase = 'go'
                self.get_logger().info(f'target {self.visited}: {label} at ({x:.2f}, {y:.2f})')
                return
        if not self.candidates:
            self.finish_goal('aborted', 'no reachable frontier or viewpoint near the goal')
            self.fire('goal_ended', 'goal aborted')
            return
        self.target = self.candidates.pop(0)
        self.plan.start(self.plan_goal(self.pose(*self.target[:3])))

    @step(Step.UNSTUCK)
    def on_unstuck(self, status):
        u = self.unstuck
        if u['phase'] == 'ask':
            # What does the global costmap have under the footprint right now?
            if u['future'] is None:
                pose = self.frame_pose(self.cfg['map_frame'])
                if pose is None or not self.get_cost.service_is_ready():
                    self.abort_stuck('cannot ask the global costmap (no pose or no '
                                     f'{self.cfg["get_cost_service"]} service)')
                    return
                request = GetCost.Request(use_footprint=True)
                request.x, request.y, request.theta = pose
                u['future'], u['pose'] = self.get_cost.call_async(request), pose
                return
            if not u['future'].done():
                return
            cost = u['future'].result().cost if u['future'].result() else None
            u['future'] = None
            if cost is None:
                self.abort_stuck('GetCost gave no answer')
                return
            if cost < INSCRIBED:
                self.fire('unstuck_done', f'global costmap under the robot is {cost:.0f} now; '
                                          'replanning')
                return
            self.decide_unstuck(cost, u['pose'])
        elif u['phase'] == 'clearing':
            if u['future'] is not None and not u['future'].done():
                return
            if time.monotonic() >= u['wait_until']:
                u['phase'], u['future'] = 'ask', None
        elif u['phase'] == 'moving':
            if self.moves is None or self.moves.poll():
                self.moves = None
                u['phase'], u['future'] = 'ask', None

    def decide_unstuck(self, cost, pose):
        """Decide whether the obstacle the costmap has under the robot is real, then act."""
        u = self.unstuck
        footprint = self.footprint
        if footprint is None:
            # The shape comes only from robot_footprint_publisher, through the costmap; without
            # it there is nothing to judge "inside the footprint" with, and no guess is made.
            self.abort_stuck(f'no footprint on {self.cfg["footprint_topic"]} yet, so the robot '
                             f'cannot check what is under it (costmap {cost:.0f})')
            return
        hits = self.scan_hits()
        grid = self.map_grid()
        in_map = (map_cells_under(grid[0], grid[1], grid[2], pose, footprint)
                  if grid is not None else -1)
        evidence = (f'costmap {cost:.0f}, {in_map} occupied /map cells under the footprint, '
                    f'{hits} scan points inside it')
        self.get_logger().warn(f'START_OCCUPIED check: {evidence}')
        if hits is None:
            self.abort_stuck(f'no recent scan to check against ({evidence})')
        elif hits >= self.cfg['scan_hits_blocked']:
            self.abort_stuck(f'the scan sees something inside the footprint ({evidence}); an '
                             'object, or the robot itself in the scan, so not moving')
        elif not u['cleared'] and in_map <= 0:
            # Nothing in the map and nothing in the scan: old obstacle-layer marks. Clearing only
            # touches the scan-built layers, never RTAB-Map's static map.
            if not self.clear_around.service_is_ready():
                self.abort_stuck(f'cannot clear the costmap ({evidence})')
                return
            u['cleared'] = True
            u['future'] = self.clear_around.call_async(ClearCostmapAroundRobot.Request(
                reset_distance=float(self.cfg['clear_around_distance'])))
            u['phase'], u['wait_until'] = 'clearing', time.monotonic() + 1.0
            self.get_logger().info('clearing the costmap around the robot (stale scan marks)')
        elif in_map <= 0:
            # Cleared already, and neither the map nor the scan has anything under the robot:
            # nothing explains the costmap, and moving would be a guess.
            self.abort_stuck(f'the costmap still has an obstacle under the robot after clearing, '
                             f'but neither /map nor the scan does ({evidence})')
        elif self.unstuck_tries >= self.cfg['max_unstuck_tries']:
            self.abort_stuck(f'still in an obstacle after {self.unstuck_tries} step-outs '
                             f'({evidence})')
        else:
            target = (step_out_target(grid[0], grid[1], grid[2], pose, footprint,
                                      self.cfg['unstuck_distance'])
                      if grid is not None else None)
            if target is None:
                self.abort_stuck(f'no spot within {self.cfg["unstuck_distance"]} m where the '
                                 f'map leaves room for the robot ({evidence})')
                return
            dx, dy = target[0] - pose[0], target[1] - pose[1]
            distance = math.hypot(dx, dy)
            if distance < 0.03:
                self.abort_stuck(f'the map leaves room for the robot here, yet the costmap does '
                                 f'not ({evidence})')
                return
            # Map-frame direction into the robot frame; the move keeps the heading.
            c, s = math.cos(pose[2]), math.sin(pose[2])
            direction = ((c * dx + s * dy) / distance, (-s * dx + c * dy) / distance)
            self.unstuck_tries += 1
            self.moves = DirectMove(self, direction, distance + 0.02, self.cfg['step_speed'])
            u['phase'] = 'moving'
            self.get_logger().info(f'stepping out {distance:.2f} m to ({target[0]:.2f}, '
                                   f'{target[1]:.2f}), try {self.unstuck_tries} ({evidence})')

    def abort_stuck(self, why):
        self.goal_code = _R.START_OCCUPIED
        self.finish_goal('aborted', f'START_OCCUPIED: {why}')
        self.fire('goal_ended', 'goal aborted')

    def scan_hits(self):
        """Count scan points inside the footprint; None without a fresh scan and its TF."""
        if self.scan is None or self.footprint is None or self.scan_problem():
            return None
        try:
            t = self.tf_buffer.lookup_transform(self.cfg['base_frame'],
                                                self.scan.header.frame_id, Time()).transform
        except tf2_ros.TransformException:
            return None
        yaw = yaw_of(t.rotation)
        m = self.scan
        points = scan_points(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max,
                             (t.translation.x, t.translation.y, yaw))
        return points_inside(points, self.footprint)

    def map_grid(self):
        """(grid, resolution, origin_xy) of /map, or None."""
        if self.map is None:
            return None
        info = self.map.info
        grid = np.asarray(self.map.data, dtype=np.int16).reshape(info.height, info.width)
        return grid, info.resolution, (info.origin.position.x, info.origin.position.y)

    # ---- helpers ------------------------------------------------------------------------------

    def pick_candidates(self):
        """Frontiers toward the goal first, then the reachable spots nearest it."""
        map_grid = self.map_grid()
        if map_grid is None:
            self.get_logger().error('no /map received, cannot pick targets')
            return []
        grid, resolution, origin = map_grid
        pose = self.frame_pose(self.cfg['map_frame'])
        if pose is None:
            self.get_logger().error(f'no robot pose in {self.cfg["map_frame"]}, cannot pick '
                                    'targets')
            return []
        here = pose[:2]
        g = self.goal.pose.position
        c = self.cfg
        out = []
        for f in find_frontiers(grid, resolution, origin, here, goal_xy=(g.x, g.y),
                                min_frontier_size=c['min_frontier_size'],
                                potential_scale=c['potential_scale'],
                                gain_scale=c['gain_scale'], goal_scale=c['goal_scale'],
                                clearance=c['robot_clearance'],
                                max_goal_distance=c['explore_radius'], exclude=self.tried,
                                exclude_radius=c['viewpoint_spacing']):
            if f.target is not None:
                out.append((*f.target, f'frontier {f.size_m:.1f} m long, '
                                       f'{f.goal_distance:.1f} m from the goal'))
        for v in find_viewpoints(grid, resolution, origin, (g.x, g.y), c['explore_radius'],
                                 c['robot_clearance'], robot_xy=here,
                                 spacing=c['viewpoint_spacing'], exclude=self.tried,
                                 max_results=c['max_target_checks']):
            if all(math.hypot(v.x - o[0], v.y - o[1]) >= c['viewpoint_spacing'] for o in out):
                out.append((v.x, v.y, v.yaw, f'viewpoint {v.distance:.1f} m from the goal'))
        return out[:c['max_target_checks']]

    def odom_pose(self):
        """(x, y, yaw) of the robot in the odom frame, or None."""
        return self.frame_pose(self.cfg['odom_frame'])

    def frame_pose(self, frame):
        """(x, y, yaw) of the robot in `frame`, or None."""
        try:
            t = self.tf_buffer.lookup_transform(frame, self.cfg['base_frame'], Time()).transform
        except tf2_ros.TransformException:
            return None
        yaw = yaw_of(t.rotation)
        return t.translation.x, t.translation.y, yaw

    def plan_error(self):
        result = self.plan.result
        if result is None:
            return 'planner unavailable'
        return 'planner: ' + describe_error(getattr(result, 'error_code', 0),
                                            getattr(result, 'error_msg', ''), self.error_names)

    def nav_error(self):
        result = self.nav.result
        if result is None:
            return 'navigate_to_pose unavailable or rejected the goal'
        code = getattr(result, 'error_code', 0)
        self.goal_code = code
        if not code:
            return f'goal status {self.nav.status}'
        return describe_error(code, getattr(result, 'error_msg', ''), self.error_names)

    def start_spin(self):
        goal = Spin.Goal()
        goal.target_yaw = 2.0 * math.pi
        goal.time_allowance.sec = 40
        self.spin.start(goal)

    def cancel_motion(self):
        for call in (self.nav, self.plan, self.spin):
            call.cancel()
        if self.moves is not None:
            self.moves.stop()
        self.moves = None

    def pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = self.cfg['map_frame']
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return msg

    def plan_goal(self, pose):
        goal = ComputePathToPose.Goal()
        goal.goal = pose
        goal.use_start = False
        return goal

    def nav_goal(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal


def main(args=None):
    # Ctrl-C as a plain KeyboardInterrupt: rclpy's own handler shuts the context down while the
    # multi-threaded executor is still waiting on it, which ends in an RCLError traceback.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    # Installed explicitly: a process started in the background inherits SIGINT as ignored, and
    # ros2 launch follows SIGINT with SIGTERM.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = MappedNavigator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.finish_goal('aborted', 'navigator shutting down')
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
