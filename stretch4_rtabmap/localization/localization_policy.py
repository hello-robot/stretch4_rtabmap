"""
Whether the robot's pose can be trusted, and which mode RTAB-Map should be in.

Used by rtabmap_supervisor.

    NO_DATA     RTAB-Map not up yet, silent, or its process exited. Goals are cancelled.
    NEED_POSE   no fix and no initial pose yet. Goals are cancelled.
    UNVERIFIED  a pose was given but not confirmed yet. Driving is allowed.
    LOCALIZED   the pose is confirmed.
    JOINING     mapping, not linked to the old map yet; nothing is written until it links.
    MAPPING     mapping, linked to the old map; driving updates it.
    LOST        the pose cannot be trusted. Goals keep running, in localization, so a closure
                or a new 2D Pose Estimate can bring it back to UNVERIFIED.

Mode switches (only when manage_mode is on):

    LOCALIZED for mapping_after s           -> set_mode_mapping   (if enable_mapping, not dark)
    MAPPING, LOST, dark                     -> set_mode_localization

The mapping wait doubles after every LOST from mapping, up to mapping_after_max.
"""

from dataclasses import dataclass
from enum import Enum
import math

from stretch4_rtabmap.localization.rtabmap_state import HINTS as SESSION_HINTS, State


class Status(str, Enum):
    NO_DATA = 'NO_DATA'
    NEED_POSE = 'NEED_POSE'
    UNVERIFIED = 'UNVERIFIED'
    LOCALIZED = 'LOCALIZED'
    JOINING = 'JOINING'
    MAPPING = 'MAPPING'
    LOST = 'LOST'


# Goals sent to nav2 are cancelled in these; everything else may drive.
BLOCKED = (Status.NO_DATA, Status.NEED_POSE)

HINTS = {
    Status.NO_DATA: SESSION_HINTS[State.NO_DATA],
    Status.NEED_POSE: 'set the pose in RViz (2D Pose Estimate)',
    Status.UNVERIFIED: 'pose not confirmed yet; send goals through mapped space and RTAB-Map '
                       'confirms it on the way',
    Status.LOCALIZED: 'pose confirmed against the map',
    Status.JOINING: 'mapping on; joins the map as the robot drives through mapped space (nothing '
                    'is written until then)',
    Status.MAPPING: 'joined to the map; driving updates it',
    Status.LOST: 'pose not trusted; still driving in localization (map not written) so a closure '
                 'can find it. A 2D Pose Estimate helps',
}


@dataclass
class Decision:
    status: Status
    detail: str = ''               # short, for people: RViz and /localization_status
    request_mapping: bool = None   # True/False: call set_mode_mapping/localization now
    event: str = ''                # logged once: why something changed
    debug: str = ''                # the numbers behind it, logged on each status change


class LocalizationPolicy:

    def __init__(self, manage_mode=True, enable_mapping=True, trust_start_pose=False,
                 lost_budget_m=6.0, mapped_lost_updates=10, mapping_after=10.0,
                 mapping_after_max=120.0, request_retry=60.0):
        self.manage_mode = manage_mode
        self.mapping_allowed = enable_mapping
        self.lost_budget_m = lost_budget_m
        self.mapped_lost_updates = mapped_lost_updates
        self.mapping_after = mapping_after
        self.mapping_after_max = mapping_after_max
        self.mapping_wait = mapping_after    # s LOCALIZED before the next switch to mapping
        self.request_retry = request_retry   # s before a mode request that did nothing is resent
        self.pose_given = trust_start_pose   # a person set the pose, or there has been a fix
        self.travel = 0.0                    # m of odometry since the last fix or pose
        self.mapped_updates = 0              # mapping updates in mapped space, no old-map closure
        self.graph_rejections = 0            # ...of which a closure was rejected by graph error
        self.blamed = ''                     # link the graph check blamed most recently
        self.rejection_logged = False        # JOINING: graph rejections already logged
        self.switching = False               # mode changed, no RTAB-Map update since (set by node)
        self.suspect = False                 # judged lost while mapping: LOST until a fix or pose
        self.relocalize = False              # a pose was given while mapping
        self.localized_since = None
        self.requested = None                # (mapping, time) of the pending mode request
        self.dark = ''                       # why mapping is held for the light (set by node)
        self.status = None

    # ---- inputs -----------------------------------------------------------------------------

    def on_update(self, mapping, in_mapped_space, moved, graph_rejected=False, blamed=''):
        """
        Count one RTAB-Map update; call before on_fix for the same update.

        Only an update in mapping, with the robot moving (not RTAB-Map's Memory/Small_movement)
        and in already-mapped space, was a chance to close onto the old map. graph_rejected: a
        closure was found and rejected by RTAB-Map's graph check; blamed: the link it blamed.
        """
        if mapping and in_mapped_space and moved:
            self.mapped_updates += 1
            if graph_rejected:
                self.graph_rejections += 1
                self.blamed = blamed or self.blamed

    def on_fix(self, to_map=True):
        """
        Record an accepted closure.

        to_map: the closure checked the pose against the map: always in localization; in mapping
        only a closure to a node of the old map (one inside this session checks nothing).
        """
        self.pose_given = True
        self.travel = 0.0
        if to_map:
            self.suspect = False
            self.mapped_updates = self.graph_rejections = 0

    def on_initial_pose(self):
        """Record a pose given in localization mode (RTAB-Map applies it)."""
        self.pose_given = True
        self.travel = 0.0
        self.mapped_updates = self.graph_rejections = 0
        self.suspect = self.relocalize = False

    def on_initial_pose_while_mapping(self):
        """Switch to localization so the pose can be applied; the node sends it again then."""
        self.relocalize = True

    def add_travel(self, metres):
        """
        Count odometry, except while RTAB-Map switches modes.

        It can match nothing then (a switch to localization takes it ~35 s), so that driving says
        nothing about the pose.
        """
        if not self.switching:
            self.travel += metres

    def request_failed(self):
        """Ask again on the next step: the mode service was not available."""
        self.requested = None

    # ---- decision ---------------------------------------------------------------------------

    def step(self, tracker, now):
        decision = self._decide(tracker, now)
        if self.dark and self.manage_mode and self.mapping_allowed and not tracker.mapping \
                and decision.status not in BLOCKED:
            decision.detail += f'; mapping off: {self.dark}'
        self.status = decision.status
        return decision

    def _decide(self, tracker, now):
        rtab = tracker.state
        if self.requested is not None and (tracker.mapping == self.requested[0]
                                           or now - self.requested[1] > self.request_retry):
            self.requested = None
        if rtab != State.LOCALIZED:
            self.localized_since = None

        if rtab in (State.WAITING, State.NO_DATA):
            silent = tracker.silent_for()
            detail = ('waiting for RTAB-Map' if rtab == State.WAITING else
                      'RTAB-Map stopped' if tracker.node_gone else
                      f'no data from RTAB-Map for {silent:.0f} s')
            return Decision(Status.NO_DATA, detail, debug=tracker.describe())

        if tracker.mapping:
            return self._decide_mapping(tracker, now)

        if rtab == State.LOCALIZED:
            if self.localized_since is None:
                self.localized_since = now
            decision = Decision(Status.LOCALIZED, 'pose confirmed', debug=tracker.describe())
            # Dark: no countdown; step() says why mapping is off.
            mapping_next = self.manage_mode and self.mapping_allowed and not self.dark
            if mapping_next and self.requested is not None:
                decision.detail = 'pose confirmed, starting mapping'
            elif mapping_next and now - self.localized_since < self.mapping_wait:
                wait = self.mapping_wait - (now - self.localized_since)
                decision.detail = f'pose confirmed, mapping in {wait:.0f} s'
            elif mapping_next:
                self.mapped_updates = self.graph_rejections = 0
                self.rejection_logged = False
                decision.detail = 'pose confirmed, starting mapping'
                decision.event = (f'switching to mapping: LOCALIZED for '
                                  f'{now - self.localized_since:.0f} s')
                return self._request(True, decision, now)
            return decision

        # NOT_LOCALIZED
        numbers = (f'{tracker.accepted}/{tracker.localized_after} fixes, {self.travel:.1f}/'
                   f'{self.lost_budget_m:.1f} m since the last fix or pose')
        if not self.pose_given:
            return Decision(Status.NEED_POSE, 'set the pose with 2D Pose Estimate',
                            debug='no initial pose and no fix yet')
        if self.relocalize:
            return Decision(Status.LOST, 'applying the new pose',
                            debug='pose given while mapping, sent again in localization')
        if self.suspect:
            return Decision(Status.LOST, 'no match with the map while mapping',
                            debug=f'lost while mapping, waiting for a closure or a pose; '
                                  f'{numbers}')
        if self.travel > self.lost_budget_m:
            return Decision(Status.LOST, f'{self.travel:.0f} m without a match', debug=numbers)
        return Decision(Status.UNVERIFIED, f'confirming the pose ({tracker.accepted}/'
                                           f'{tracker.localized_after})', debug=numbers)

    def _decide_mapping(self, tracker, now):
        rtab = tracker.state
        if not self.manage_mode:
            return Decision(Status.MAPPING, 'mapping', debug=tracker.describe())
        if not self.mapping_allowed:
            return self._request(False, Decision(
                Status.JOINING, 'switching to localization',
                event='enable_mapping is false; switching RTAB-Map to localization'), now)
        if self.relocalize:
            return self._request(False, Decision(
                Status.LOST, 'applying the new pose',
                event='2D Pose Estimate while mapping; switching to localization to apply it'),
                now)
        if self.dark and not self.suspect:   # LOST keeps its own status
            status = Status.MAPPING if rtab == State.MAPPING_LINKED else Status.JOINING
            event = (f'{self.dark}; switching RTAB-Map to localization'
                     if self.requested is None else '')
            return self._request(False, Decision(
                status, f'{self.dark}, switching to localization', event=event), now)
        if rtab != State.MAPPING_LINKED and not self.suspect:
            # JOINING: RTAB-Map writes nothing until the link, so stay in mapping and keep
            # trying; going back to localization only costs a ~35 s switch.
            link = self.blamed or 'link not reported'
            event = ''
            if self.graph_rejections and not self.rejection_logged:
                self.rejection_logged = True
                event = (f'joining: RTAB-Map finds the old map but its graph check '
                         f'(RGBD/OptimizeMaxError) rejects the closures, worst {link}; staying in '
                         'mapping and trying on')
            debug = (f'{self.graph_rejections} closures rejected by the graph check (worst '
                     f'{link}); still trying' if self.graph_rejections else
                     f'{self.mapped_updates} updates in mapped space, no link yet')
            detail = ('joining the map, matches rejected so far' if self.graph_rejections
                      else 'joining the map')
            return Decision(Status.JOINING, detail, event=event, debug=debug)
        attempts = f'{self.mapped_updates}/{self.mapped_lost_updates} updates in mapped space ' \
                   'without an old-map closure'
        event = ''
        if not self.suspect and self.mapped_updates >= self.mapped_lost_updates:
            self.suspect = True
            self.mapping_wait = min(self.mapping_wait * 2.0, self.mapping_after_max)
            event = (f'lost while mapping: {attempts}; switching to localization, next try at '
                     f'mapping after {self.mapping_wait:.0f} s LOCALIZED')
        if self.suspect:
            return self._request(False, Decision(
                Status.LOST, 'no match with the map, back to localization', event=event,
                debug=f'{attempts}; switching to localization (map not written)'), now)
        if self.mapping_wait != self.mapping_after:
            self.mapping_wait = self.mapping_after
        return Decision(Status.MAPPING, 'updating the map',
                        debug=f'{tracker.describe()}; {attempts}')

    def _request(self, mapping, decision, now):
        """Ask for a mode once; callers only ask for the mode RTAB-Map is not in."""
        if self.requested is None:
            self.requested = (mapping, now)
            decision.request_mapping = mapping
        return decision


class LightCheck:
    """
    Whether it is too dark to map, from the camera image's mean brightness (0-255).

    Dark after dark_after s below dark_below; light again after light_after s above light_above.
    The first image outside the two thresholds decides at once; until then, and with no image for
    stale_after s, it counts as dark.
    """

    def __init__(self, dark_below=40.0, light_above=70.0, dark_after=5.0, light_after=30.0,
                 stale_after=5.0):
        self.dark_below = dark_below
        self.light_above = light_above
        self.dark_after = dark_after
        self.light_after = light_after
        self.stale_after = stale_after
        self.dark = None                     # None: not known yet
        self.brightness = None
        self.seen_at = None
        self.since = None                    # when the brightness crossed the other threshold

    def update(self, brightness, now):
        """Add one image's brightness; return an event when dark or light changes."""
        self.brightness, self.seen_at = brightness, now
        if self.dark is None and not self.dark_below <= brightness <= self.light_above:
            self.dark = brightness < self.dark_below
            state = 'dark' if self.dark else 'light'
            return f'{state} at start: camera brightness {brightness:.0f}'
        dark = self.dark is not False
        crossed = brightness > self.light_above if dark else brightness < self.dark_below
        if not crossed:
            self.since = None
            return ''
        if self.since is None:
            self.since = now
        if now - self.since < (self.light_after if dark else self.dark_after):
            return ''
        self.dark, self.since = not dark, None
        return f'{"dark" if self.dark else "light"}: camera brightness {brightness:.0f}'

    def reason(self, now):
        """Return why mapping is held, or '' when it is light."""
        if self.seen_at is None or now - self.seen_at > self.stale_after:
            return 'no camera image'
        if self.dark is not False and self.brightness > self.light_above:
            return f'waiting for steady light ({self.brightness:.0f})'
        if self.dark is not False:
            return f'too dark ({self.brightness:.0f})'
        return ''


def compose(a, b):
    """Return the 2D pose (x, y, yaw) b expressed in a's parent frame (a * b)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    """Return the inverse of the 2D pose a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def carry_pose(pose, odom_then, odom_now):
    """
    Return where a map pose given at one time is now, after driving on odometry.

    pose: (x, y, yaw) in the map when it was given; odom_then, odom_now: odom -> base then and now.
    """
    return compose(pose, compose(inverse(odom_then), odom_now))
