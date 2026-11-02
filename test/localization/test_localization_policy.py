from stretch4_rtabmap.localization.localization_policy import (
    BLOCKED, LightCheck, LocalizationPolicy, Status)
from stretch4_rtabmap.localization.rtabmap_state import DISTANCE_KEY, SessionTracker


def setup(mode_mapping=False, **kwargs):
    tracker = SessionTracker(localized_after=2, lost_after_m=3.0)
    tracker.gate = True
    tracker.set_mode(mode_mapping)
    tracker.update(10, 0, 0, {DISTANCE_KEY: 0.0})
    return tracker, LocalizationPolicy(**kwargs)


def fix(tracker, policy, node, target):
    tracker.update(node, target, 0, {DISTANCE_KEY: 0.0})
    policy.on_fix()


def test_no_data_until_rtabmap_reports():
    policy = LocalizationPolicy()
    assert policy.step(SessionTracker(), 0.0).status == Status.NO_DATA


def test_needs_a_pose_then_drives_unverified():
    tracker, policy = setup()
    decision = policy.step(tracker, 0.0)
    assert decision.status == Status.NEED_POSE
    tracker.on_initial_pose()
    policy.on_initial_pose()
    assert policy.step(tracker, 1.0).status == Status.UNVERIFIED


def test_trusted_start_pose_skips_need_pose():
    tracker, policy = setup(trust_start_pose=True)
    assert policy.step(tracker, 0.0).status == Status.UNVERIFIED


def test_lost_after_budget_and_back_on_pose():
    tracker, policy = setup(lost_budget_m=6.0)
    policy.on_initial_pose()
    policy.add_travel(5.0)
    assert policy.step(tracker, 0.0).status == Status.UNVERIFIED
    policy.add_travel(1.5)
    decision = policy.step(tracker, 1.0)
    assert decision.status == Status.LOST
    assert Status.LOST not in BLOCKED                  # keeps driving so a closure can find it
    assert decision.request_mapping is None            # and never maps while lost
    policy.on_initial_pose()
    assert policy.step(tracker, 3.0).status == Status.UNVERIFIED


def test_lost_recovers_on_a_closure():
    tracker, policy = setup(lost_budget_m=6.0, mapping_after=0.0)
    policy.on_initial_pose()
    policy.add_travel(7.0)
    assert policy.step(tracker, 0.0).status == Status.LOST
    fix(tracker, policy, 11, 5)
    assert policy.step(tracker, 1.0).status == Status.UNVERIFIED
    fix(tracker, policy, 12, 6)
    assert policy.step(tracker, 2.0).status == Status.LOCALIZED


def test_a_fix_resets_the_budget():
    tracker, policy = setup()
    policy.on_initial_pose()
    policy.add_travel(5.9)
    fix(tracker, policy, 11, 5)
    policy.add_travel(5.9)
    assert policy.step(tracker, 0.0).status == Status.UNVERIFIED


def test_localized_then_mapping_after_hold():
    tracker, policy = setup(mapping_after=10.0)
    fix(tracker, policy, 11, 5)
    fix(tracker, policy, 12, 6)
    first = policy.step(tracker, 0.0)
    assert first.status == Status.LOCALIZED and first.request_mapping is None
    decision = policy.step(tracker, 10.0)
    assert decision.request_mapping is True
    assert policy.step(tracker, 10.2).request_mapping is None   # asked once
    tracker.set_mode(True)
    tracker.update(100, 0, 0, {})
    assert policy.step(tracker, 11.0).status == Status.JOINING
    tracker.update(101, 0, 40, {})
    assert policy.step(tracker, 12.0).status == Status.MAPPING


def test_no_mapping_when_disabled():
    tracker, policy = setup(enable_mapping=False, mapping_after=0.0)
    fix(tracker, policy, 11, 5)
    fix(tracker, policy, 12, 6)
    assert policy.step(tracker, 100.0).request_mapping is None


def test_report_only_never_switches():
    tracker, policy = setup(manage_mode=False, mapping_after=0.0)
    fix(tracker, policy, 11, 5)
    fix(tracker, policy, 12, 6)
    assert policy.step(tracker, 100.0).request_mapping is None
    tracker.set_mode(True)
    tracker.update(100, 0, 0, {})
    assert policy.step(tracker, 101.0).status == Status.MAPPING


def joining(**kwargs):
    """LOCALIZED, switched to mapping, not linked yet (session starts at node 100)."""
    tracker, policy = setup(mapped_lost_updates=10, mapping_after=10.0, **kwargs)
    fix(tracker, policy, 11, 5)
    fix(tracker, policy, 12, 6)
    policy.step(tracker, 0.0)
    assert policy.step(tracker, 10.0).request_mapping is True
    tracker.set_mode(True)
    tracker.update(100, 0, 0, {})
    return tracker, policy


def mapped_updates(tracker, policy, count, node=200, in_mapped_space=True, moved=True):
    for i in range(count):
        tracker.update(node + i, 0, 0, {})
        policy.on_update(True, in_mapped_space, moved)


def test_joining_has_no_time_or_distance_limit():
    tracker, policy = joining()
    policy.add_travel(100.0)
    mapped_updates(tracker, policy, 50, in_mapped_space=False)    # new space never counts
    mapped_updates(tracker, policy, 50, moved=False)              # standing still never counts
    decision = policy.step(tracker, 5000.0)
    assert decision.status == Status.JOINING and decision.request_mapping is None


def test_joining_links_after_slow_updates():
    tracker, policy = joining()
    mapped_updates(tracker, policy, 9)
    assert policy.step(tracker, 100.0).status == Status.JOINING
    tracker.update(300, 0, 5, {})                                 # link to the old map
    policy.on_update(True, True, True)
    policy.on_fix(to_map=True)
    assert policy.step(tracker, 101.0).status == Status.MAPPING
    assert policy.mapped_updates == 0


def test_mapping_switched_off_when_not_allowed():
    tracker, policy = setup(mode_mapping=True, enable_mapping=False)
    tracker.update(100, 0, 0, {})
    assert policy.step(tracker, 0.0).request_mapping is False


def test_failed_request_is_retried():
    tracker, policy = setup(mapping_after=0.0)
    fix(tracker, policy, 11, 5)
    fix(tracker, policy, 12, 6)
    assert policy.step(tracker, 0.0).request_mapping is True
    policy.request_failed()
    assert policy.step(tracker, 0.2).request_mapping is True


def linked_mapping(**kwargs):
    tracker, policy = setup(mode_mapping=True, mapped_lost_updates=10, **kwargs)
    policy.on_initial_pose()
    tracker.update(101, 0, 5, {})             # link to the old map (session started at 10)
    policy.on_fix(to_map=True)
    return tracker, policy


def test_new_space_never_lost_while_mapping():
    tracker, policy = linked_mapping()
    mapped_updates(tracker, policy, 100, in_mapped_space=False)
    assert policy.step(tracker, 0.0).status == Status.MAPPING


def test_lost_in_mapped_space_switches_to_localization():
    tracker, policy = linked_mapping()
    mapped_updates(tracker, policy, 8)
    policy.on_fix(to_map=False)                # a closure inside this session checks nothing
    mapped_updates(tracker, policy, 2, node=300)
    decision = policy.step(tracker, 0.0)
    assert decision.status == Status.LOST and decision.request_mapping is False
    assert policy.step(tracker, 0.2).request_mapping is None      # asked once
    tracker.set_mode(False)
    tracker.update(400, 0, 0, {})
    assert policy.step(tracker, 40.0).status == Status.LOST       # until a fix or a pose
    fix(tracker, policy, 401, 5)
    assert policy.step(tracker, 41.0).status == Status.UNVERIFIED


def test_old_map_closure_resets_mapped_updates():
    tracker, policy = linked_mapping()
    mapped_updates(tracker, policy, 8)
    policy.on_fix(to_map=True)
    mapped_updates(tracker, policy, 8, node=300)
    assert policy.step(tracker, 0.0).status == Status.MAPPING


def test_pose_while_mapping_switches_to_localization():
    tracker, policy = linked_mapping()
    policy.on_initial_pose_while_mapping()
    decision = policy.step(tracker, 0.0)
    assert decision.status == Status.LOST and decision.request_mapping is False
    tracker.set_mode(False)
    assert policy.step(tracker, 39.0).status == Status.LOST       # until the pose is applied
    policy.on_initial_pose()                   # the supervisor sends the held pose again
    assert policy.step(tracker, 40.0).status == Status.UNVERIFIED


def test_carry_pose_follows_odometry():
    import math
    from stretch4_rtabmap.localization.localization_policy import carry_pose
    # Given at map (5, 0) facing +y; then drove 1 m forward on odometry (odom facing +x).
    x, y, yaw = carry_pose((5.0, 0.0, math.pi / 2), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert abs(x - 5.0) < 1e-9 and abs(y - 1.0) < 1e-9 and abs(yaw - math.pi / 2) < 1e-9


def graph_rejected_updates(tracker, policy, count, node=600, link='link 12089->11209 x94'):
    for i in range(count):
        tracker.update(node + i, 0, 0, {})
        policy.on_update(True, True, True, graph_rejected=True, blamed=link)


def joined():
    """Linked mapping, reached through JOINING (mapping wait 10 s)."""
    tracker, policy = joining()
    tracker.update(300, 0, 5, {})                                 # link to the old map
    policy.on_update(True, True, True)
    policy.on_fix(to_map=True)
    assert policy.step(tracker, 20.0).status == Status.MAPPING
    return tracker, policy


def test_joining_never_gives_up_even_when_the_map_rejects():
    tracker, policy = joining()
    graph_rejected_updates(tracker, policy, 200)
    mapped_updates(tracker, policy, 200, node=900)
    first = policy.step(tracker, 100.0)
    assert first.status == Status.JOINING and first.request_mapping is None
    assert '200 closures rejected by the graph check' in first.debug and first.event
    assert first.detail == 'joining the map, matches rejected so far'
    later = policy.step(tracker, 5000.0)
    assert later.status == Status.JOINING and not later.event           # logged once


def test_mapping_lost_after_updates_then_retries_with_backoff():
    tracker, policy = joined()
    mapped_updates(tracker, policy, 10, node=400)
    decision = policy.step(tracker, 100.0)
    assert decision.status == Status.LOST and decision.request_mapping is False
    assert 'next try at mapping after 20 s' in decision.event
    tracker.set_mode(False)
    tracker.update(500, 0, 0, {})
    assert policy.step(tracker, 140.0).status == Status.LOST      # until a fix or a pose
    fix(tracker, policy, 501, 5)
    fix(tracker, policy, 502, 6)
    assert policy.step(tracker, 141.0).status == Status.LOCALIZED
    assert policy.step(tracker, 151.0).request_mapping is None    # waits 20 s now, not 10
    assert policy.step(tracker, 161.0).request_mapping is True    # never given up


def test_backoff_capped_and_reset_by_a_link():
    tracker, policy = joined()
    policy.mapping_wait = 80.0
    mapped_updates(tracker, policy, 10, node=400)
    policy.step(tracker, 100.0)
    assert policy.mapping_wait == 120.0                           # 160 capped to 120


def test_mapping_lost_also_when_the_graph_check_rejects():
    tracker, policy = joined()
    graph_rejected_updates(tracker, policy, 10, node=400)
    decision = policy.step(tracker, 100.0)
    assert decision.status == Status.LOST and decision.request_mapping is False


def test_no_lost_from_driving_during_a_mode_switch():
    tracker, policy = joined()
    mapped_updates(tracker, policy, 10, node=400)
    assert policy.step(tracker, 100.0).status == Status.LOST
    fix(tracker, policy, 450, 5)                  # (in mapping: does not clear suspect)
    tracker.set_mode(False)                       # parameter flips; RTAB-Map busy ~35 s
    policy.on_initial_pose()                      # a pose: trusted again
    policy.switching = True
    policy.add_travel(8.0)                        # robot keeps driving meanwhile
    assert policy.travel == 0.0
    policy.switching = False
    tracker.update(800, 0, 0, {})
    assert policy.step(tracker, 136.0).status == Status.UNVERIFIED


def test_light_check_first_image_decides():
    light = LightCheck(dark_below=40.0, light_above=70.0)
    assert light.reason(0.0) == 'no camera image'
    assert light.update(120.0, 0.0).startswith('light at start')
    assert light.reason(0.0) == ''                  # no wait when it starts in the light
    light = LightCheck(dark_below=40.0, light_above=70.0)
    assert light.update(10.0, 0.0).startswith('dark at start')
    light = LightCheck(dark_below=40.0, light_above=70.0)
    assert light.update(55.0, 0.0) == ''            # in between: dark until a clear image
    assert light.reason(0.0) == 'too dark (55)'
    assert light.update(120.0, 1.0).startswith('light at start')


def test_light_check_hysteresis():
    light = LightCheck(dark_below=40.0, light_above=70.0, dark_after=5.0, light_after=30.0)
    light.update(120.0, 0.0)
    light.update(10.0, 50.0)
    assert light.reason(54.0) == ''                 # dark for less than dark_after
    assert light.update(10.0, 55.0).startswith('dark')
    assert light.reason(55.0) == 'too dark (10)'
    light.update(120.0, 56.0)
    assert light.reason(56.0).startswith('waiting for steady light')
    light.update(50.0, 60.0)                        # between the thresholds: the wait restarts
    light.update(120.0, 61.0)
    assert light.update(120.0, 90.0) == ''
    assert light.update(120.0, 91.0).startswith('light')
    assert light.reason(97.0) == 'no camera image'  # no image for stale_after s


def test_no_mapping_in_the_dark():
    tracker, policy = setup(mapping_after=10.0)
    fix(tracker, policy, 11, 5)
    fix(tracker, policy, 12, 6)
    policy.dark = 'too dark (10)'
    decision = policy.step(tracker, 100.0)
    assert decision.status == Status.LOCALIZED and decision.request_mapping is None
    assert decision.detail == 'pose confirmed; mapping off: too dark (10)'   # no countdown
    policy.dark = ''
    assert policy.step(tracker, 110.0).request_mapping is True   # light: the usual wait


def test_dark_while_mapping_goes_straight_to_localization():
    tracker, policy = setup(mode_mapping=True, mapping_after=10.0, trust_start_pose=True)
    policy.dark = 'too dark (10)'
    decision = policy.step(tracker, 0.0)
    assert decision.request_mapping is False and decision.event
    assert 'too dark' in decision.detail
    again = policy.step(tracker, 0.2)
    assert again.request_mapping is None and not again.event   # asked and logged once
    tracker.set_mode(False)
    after = policy.step(tracker, 40.0)
    assert after.status == Status.UNVERIFIED                    # not LOST
    assert after.detail.endswith('mapping off: too dark (10)')
    assert policy.mapping_wait == 10.0                          # the wait does not double


def test_dark_while_lost_from_mapping_stays_lost():
    tracker, policy = linked_mapping()
    mapped_updates(tracker, policy, 10)
    assert policy.step(tracker, 0.0).status == Status.LOST
    policy.dark = 'too dark (10)'
    assert policy.step(tracker, 0.2).status == Status.LOST
    tracker.set_mode(False)
    decision = policy.step(tracker, 40.0)
    assert decision.status == Status.LOST
    assert decision.detail.endswith('mapping off: too dark (10)')
