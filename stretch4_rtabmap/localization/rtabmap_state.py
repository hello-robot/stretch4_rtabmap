"""
Which state an RTAB-Map session is in, from /rtabmap/info, its mode and /initialpose.

Shared by rtabmap_loop_monitor and the supervisor.

    WAITING_FOR_RTABMAP   nothing on /rtabmap/info yet, or the mode is not known.
    NOT_LOCALIZED         localization mode, no recent accepted closure.
    LOCALIZED             localization mode, enough recent closures.
    MAPPING_NOT_LINKED    mapping mode, not joined to the old map yet.
    MAPPING_LINKED        mapping mode, joined to the old map.
    NO_RTABMAP_DATA       no update for `stale_after` seconds, or the node is gone.
"""

from dataclasses import dataclass, field
from enum import Enum
import time

DISTANCE_KEY = 'Loop/Distance_since_last_loc/m'
REJECTED_KEY = 'Loop/RejectedHypothesis/'
HYPOTHESIS_KEY = 'Loop/Highest_hypothesis_id/'
INLIERS_KEY = 'Loop/Visual_inliers/'
MAX_ERROR_RATIO_KEY = 'Loop/Optimization_max_error_ratio/'
MAX_ANG_ERROR_RATIO_KEY = 'Loop/Optimization_max_ang_error_ratio/'


class State(str, Enum):
    WAITING = 'WAITING_FOR_RTABMAP'
    NOT_LOCALIZED = 'NOT_LOCALIZED'
    LOCALIZED = 'LOCALIZED'
    MAPPING_NOT_LINKED = 'MAPPING_NOT_LINKED'
    MAPPING_LINKED = 'MAPPING_LINKED'
    NO_DATA = 'NO_RTABMAP_DATA'


# What a person watching should do in each state.
HINTS = {
    State.WAITING: 'start the navigation launch; nothing from RTAB-Map yet',
    State.NOT_LOCALIZED: 'set the pose in RViz (2D Pose Estimate) and drive slowly through mapped '
                         'space',
    State.LOCALIZED: 'pose verified against the map; safe to navigate',
    State.MAPPING_NOT_LINKED: 'mapping, not joined to the old map yet; nothing is written until '
                              'it links',
    State.MAPPING_LINKED: 'joined to the old map; new nodes extend and update it',
    State.NO_DATA: 'RTAB-Map is processing nothing.',
}


@dataclass
class Update:
    """What one /rtabmap/info message changed."""

    state: State
    changed: bool
    events: list = field(default_factory=list)


class SessionTracker:
    """Track the session state across mode switches, initial poses and info updates."""

    def __init__(self, localized_after=2, lost_after_m=3.0, stale_after=60.0):
        self.localized_after = localized_after
        self.lost_after_m = lost_after_m
        self.stale_after = stale_after   # s without an update before NO_DATA; 0 disables
        self.last_update = None      # time.monotonic() of the latest /rtabmap/info
        self.stale = False
        self.node_seen = False       # the RTAB-Map node has been in the ROS graph at some point
        self.node_gone = False       # ...and is not any more: the process exited
        self.mapping = None          # None until the mode is known
        self.gate = None             # Rtabmap/StartNewMapOnLoopClosure, None until known
        self.state = State.WAITING
        self.accepted = 0            # closures accepted since entering localization / last pose
        self.distance = 0.0          # Loop/Distance_since_last_loc of the latest update
        self.session_start = None    # first node id seen in this mapping session
        self.session_updates = 0
        self.old_map_links = []      # (from_id, to_id, kind)
        self.last_rejection = None   # (hypothesis id, inliers, error ratio) of the latest one

    def set_mode(self, mapping):
        """Return True if the mode changed. Every switch starts the state over."""
        if mapping == self.mapping:
            return False
        self.mapping = mapping
        self.accepted = 0
        self.session_start, self.session_updates, self.old_map_links = None, 0, []
        self._recompute()
        return True

    def on_initial_pose(self):
        """In localization mode a new initial pose discards the previous fix."""
        if self.mapping is False:
            self.accepted = 0
            self._recompute()

    def check_stale(self, now=None):
        """Call periodically. Return True if the state changed (updates stopped)."""
        now = time.monotonic() if now is None else now
        if self.stale or not self.stale_after or self.last_update is None:
            return False
        if now - self.last_update <= self.stale_after:
            return False
        self.stale = True
        previous = self.state
        self._recompute()
        return self.state != previous

    def set_node_alive(self, alive):
        """
        Report whether the RTAB-Map node is in the ROS graph. Return True if the state changed.

        Only a node seen before and now missing counts as gone, so a slow start is not a crash.
        """
        previous = self.state
        if alive:
            self.node_seen = True
            self.node_gone = False
        elif self.node_seen:
            self.node_gone = True
        self._recompute()
        return self.state != previous

    def silent_for(self, now=None):
        """Seconds since the latest update, or None before the first one."""
        if self.last_update is None:
            return None
        return (time.monotonic() if now is None else now) - self.last_update

    def update(self, node_id, loop_closure_id, proximity_id, stats, now=None):
        events = []
        self.last_update = time.monotonic() if now is None else now
        self.node_seen, self.node_gone = True, False     # it just published, so it exists
        if self.stale:
            self.stale = False
            events.append('RTAB-Map updates resumed')
            if self.mapping is False:
                # The pose was not checked during the gap: earlier fixes no longer vouch for it.
                self.accepted = 0
        self.distance = stats.get(DISTANCE_KEY, self.distance)

        if self.mapping and node_id > 0:
            if self.session_start is None:
                self.session_start = node_id
            self.session_updates += 1

        for target, kind in ((loop_closure_id, 'LOOP'), (proximity_id, 'PROXIMITY')):
            if target <= 0:
                continue
            if self.mapping and self.session_start is not None:
                old = target < self.session_start
                if old:
                    self.old_map_links.append((node_id, target, kind))
                events.append(f'{kind} {node_id}->{target} '
                              f'({"OLD MAP" if old else "this session"})')
            else:
                events.append(f'{kind} {node_id}->{target} (map)')
        if loop_closure_id > 0 or proximity_id > 0:
            self.accepted += 1
            inliers = int(stats.get(INLIERS_KEY, 0))
            if inliers:
                events.append(f'{inliers} inliers')

        if stats.get(REJECTED_KEY, 0.0) > 0.0:
            ratio = max(stats.get(MAX_ERROR_RATIO_KEY, 0.0),
                        stats.get(MAX_ANG_ERROR_RATIO_KEY, 0.0))
            self.last_rejection = (int(stats.get(HYPOTHESIS_KEY, 0)),
                                   int(stats.get(INLIERS_KEY, 0)), ratio)
            text = f'rejected hypothesis {self.last_rejection[0]}'
            if self.last_rejection[1]:
                text += f' ({self.last_rejection[1]} inliers'
                text += f', graph error ratio {ratio:.1f})' if ratio else ')'
            events.append(text)

        previous = self.state
        self._recompute()
        return Update(self.state, self.state != previous, events)

    def describe(self):
        """One line of detail for the current state, for logs and the status topic."""
        if self.state == State.NO_DATA:
            silent = self.silent_for()
            since = f', no update for {silent:.0f} s' if silent is not None else ''
            if self.node_gone:
                return f'RTAB-Map process has exited (node not in the ROS graph{since})'
            return f'no RTAB-Map update for {silent:.0f} s'
        if self.state == State.LOCALIZED:
            return f'{self.distance:.1f} m since the last fix'
        if self.state == State.NOT_LOCALIZED:
            return (f'{self.accepted}/{self.localized_after} fixes, '
                    f'{self.distance:.1f} m since the last one')
        if self.state == State.MAPPING_LINKED:
            first = self.old_map_links[0]
            return (f'{len(self.old_map_links)} links to the old map, first {first[2]} '
                    f'{first[0]}->{first[1]}')
        if self.state == State.MAPPING_NOT_LINKED:
            if self.gate is False:
                return (f'ISLAND: {self.session_updates} updates recorded with no link, the '
                        'loop-closure gate is OFF')
            return (f'{self.session_updates} updates, new nodes discarded by the loop-closure '
                    'gate, map unchanged')
        return ''

    def _recompute(self):
        if self.node_gone or (self.stale and self.mapping is not None):
            self.state = State.NO_DATA
        elif self.mapping is None:
            self.state = State.WAITING
        elif self.mapping:
            self.state = State.MAPPING_LINKED if self.old_map_links else State.MAPPING_NOT_LINKED
        else:
            if self.distance > self.lost_after_m:
                # Too far on odometry alone: earlier fixes no longer vouch for the pose.
                self.accepted = 0
            self.state = (State.LOCALIZED if self.accepted >= self.localized_after
                          else State.NOT_LOCALIZED)
