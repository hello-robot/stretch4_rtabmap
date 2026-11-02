"""
Show which state RTAB-Map is in, what to do about it, and every closure as it happens.

    ros2 run stretch4_rtabmap rtabmap_loop_monitor

Read from /rtabmap/info:

  * loop_closure_id         global (bag-of-words) loop closure accepted on this update, 0 if none
  * proximity_detection_id  proximity (local, by space) closure accepted on this update, 0 if none
  * Loop/RejectedHypothesis a hypothesis was found but rejected: too few inliers, or the graph
                            error ratio (Loop/Optimization_max_error_ratio) above
                            RGBD/OptimizeMaxError

The mode comes from Mem/IncrementalMemory on /rtabmap/rtabmap, read at startup and watched on
/parameter_events; Rtabmap/StartNewMapOnLoopClosure is read the same way so an unlinked mapping
session can be told apart from an island being recorded.
"""

from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rtabmap_msgs.msg import Info
from std_msgs.msg import String

from stretch4_rtabmap.localization.rtabmap_state import HINTS, SessionTracker, State

MODE_PARAM = 'Mem/IncrementalMemory'
GATE_PARAM = 'Rtabmap/StartNewMapOnLoopClosure'

# Banner colour per state, so the state is visible at a glance in a scrolling terminal.
COLOURS = {
    State.WAITING: '\033[90m',
    State.NOT_LOCALIZED: '\033[93m',
    State.LOCALIZED: '\033[92m',
    State.MAPPING_NOT_LINKED: '\033[91m',
    State.MAPPING_LINKED: '\033[96m',
    State.NO_DATA: '\033[41m',
}
RESET = '\033[0m'


def node_in_graph(node, fqn):
    """Return True if the node named fqn (e.g. /rtabmap/rtabmap) is in the ROS graph."""
    namespace, _, name = fqn.rpartition('/')
    return (name, namespace or '/') in node.get_node_names_and_namespaces()


def is_true(value):
    """Return a ParameterValue as a bool; rtabmap declares its parameters as strings."""
    return value.bool_value or value.string_value.lower() == 'true'


class RtabmapLoopMonitor(Node):

    def __init__(self):
        super().__init__('rtabmap_loop_monitor')
        self.declare_parameter('rtabmap_node', '/rtabmap/rtabmap')
        self.declare_parameter('info_topic', '/rtabmap/info')
        self.declare_parameter('localized_after', 2)
        self.declare_parameter('lost_after_m', 3.0)
        self.declare_parameter('stale_after', 60.0)
        self.rtabmap_node = self.get_parameter('rtabmap_node').value

        self.tracker = SessionTracker(
            localized_after=self.get_parameter('localized_after').value,
            lost_after_m=self.get_parameter('lost_after_m').value,
            stale_after=self.get_parameter('stale_after').value)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/rtabmap/session_state', latched)

        self.create_subscription(
            Info, self.get_parameter('info_topic').value, self.on_info, 10)
        self.create_subscription(ParameterEvent, '/parameter_events', self.on_param_event, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10)
        self.get_params = self.create_client(GetParameters, f'{self.rtabmap_node}/get_parameters')
        self.params_known = False
        self.create_timer(1.0, self.query_params)
        self.create_timer(1.0, self.check_stale)
        self.announce()

    def query_params(self):
        if self.params_known or not self.get_params.service_is_ready():
            return
        future = self.get_params.call_async(
            GetParameters.Request(names=[MODE_PARAM, GATE_PARAM]))
        future.add_done_callback(self.on_params_reply)

    def on_params_reply(self, future):
        values = future.result().values if future.result() else []
        if len(values) != 2 or self.params_known:
            return
        self.params_known = True
        self.tracker.gate = is_true(values[1])
        if self.tracker.set_mode(is_true(values[0])):
            self.announce()

    def on_param_event(self, event):
        if event.node != self.rtabmap_node:
            return
        for param in list(event.new_parameters) + list(event.changed_parameters):
            if param.name == GATE_PARAM:
                self.tracker.gate = is_true(param.value)
            elif param.name == MODE_PARAM and self.tracker.set_mode(is_true(param.value)):
                self.announce()

    def check_stale(self):
        alive_changed = self.tracker.set_node_alive(node_in_graph(self, self.rtabmap_node))
        if self.tracker.check_stale() or alive_changed:
            self.announce()
        elif self.tracker.state == State.NO_DATA:
            self.get_logger().error(f'[{State.NO_DATA.value}] {self.tracker.describe()}',
                                    throttle_duration_sec=10.0)

    def on_initial_pose(self, _msg):
        before = self.tracker.state
        self.tracker.on_initial_pose()
        if self.tracker.state != before:
            self.announce('new initial pose')

    def on_info(self, info):
        update = self.tracker.update(
            info.ref_id, info.loop_closure_id, info.proximity_detection_id,
            dict(zip(info.stats_keys, info.stats_values)))
        if update.changed:
            self.announce('updates resumed' if 'RTAB-Map updates resumed' in update.events
                          else '')

        state = self.tracker.state
        line = f'[{state.value}] node {info.ref_id} | {self.tracker.describe()}'
        if update.events:
            line += ' | ' + ', '.join(update.events)
        # rclpy fixes the severity per call site, so warn and info need separate calls.
        if state in (State.NOT_LOCALIZED, State.MAPPING_NOT_LINKED):
            self.get_logger().warn(line)
        else:
            self.get_logger().info(line)
        self.publish_state()

    def announce(self, reason=''):
        state = self.tracker.state
        colour = COLOURS[state]
        detail = self.tracker.describe()
        lines = [
            f'{colour}==== {state.value} ===={RESET}' + (f'  ({reason})' if reason else ''),
            f'{colour}     {HINTS[state]}{RESET}',
        ]
        if detail:
            lines.append(f'     {detail}')
        self.get_logger().info('\n'.join(lines))
        self.publish_state()

    def publish_state(self):
        detail = self.tracker.describe()
        self.state_pub.publish(String(
            data=self.tracker.state.value + (f' | {detail}' if detail else '')))


def main(args=None):
    rclpy.init(args=args)
    node = RtabmapLoopMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
