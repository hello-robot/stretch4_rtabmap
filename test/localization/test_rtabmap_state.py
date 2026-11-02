from stretch4_rtabmap.localization.rtabmap_state import (
    DISTANCE_KEY, REJECTED_KEY, SessionTracker, State)


def localization_tracker():
    tracker = SessionTracker(localized_after=2, lost_after_m=3.0)
    tracker.gate = True
    tracker.set_mode(False)
    return tracker


def test_waiting_until_mode_known():
    tracker = SessionTracker()
    assert tracker.update(10, 0, 0, {}).state == State.WAITING


def test_localized_after_two_fixes():
    tracker = localization_tracker()
    assert tracker.update(10, 5, 0, {DISTANCE_KEY: 0.0}).state == State.NOT_LOCALIZED
    update = tracker.update(11, 0, 6, {DISTANCE_KEY: 0.0})
    assert update.state == State.LOCALIZED and update.changed


def test_lost_after_distance_without_fix():
    tracker = localization_tracker()
    tracker.update(10, 5, 0, {})
    tracker.update(11, 6, 0, {})
    assert tracker.update(12, 0, 0, {DISTANCE_KEY: 3.5}).state == State.NOT_LOCALIZED
    # Earlier fixes no longer count: two fresh ones are needed.
    assert tracker.update(13, 7, 0, {DISTANCE_KEY: 0.0}).state == State.NOT_LOCALIZED


def test_initial_pose_discards_fix():
    tracker = localization_tracker()
    tracker.update(10, 5, 0, {})
    tracker.update(11, 6, 0, {})
    tracker.on_initial_pose()
    assert tracker.state == State.NOT_LOCALIZED


def test_mapping_links_only_on_closure_to_old_node():
    tracker = localization_tracker()
    tracker.set_mode(True)
    assert tracker.update(100, 0, 0, {}).state == State.MAPPING_NOT_LINKED
    # A closure inside the new session is not a link to the old map.
    assert tracker.update(101, 100, 0, {}).state == State.MAPPING_NOT_LINKED
    update = tracker.update(102, 0, 40, {})
    assert update.state == State.MAPPING_LINKED and update.changed
    assert 'OLD MAP' in update.events[0]


def test_switch_back_to_localization_needs_new_fixes():
    tracker = localization_tracker()
    tracker.update(10, 5, 0, {})
    tracker.update(11, 6, 0, {})
    tracker.set_mode(True)
    tracker.update(12, 6, 0, {})
    tracker.set_mode(False)
    assert tracker.state == State.NOT_LOCALIZED


def test_rejection_reports_graph_error_ratio():
    tracker = localization_tracker()
    update = tracker.update(10, 0, 0, {
        REJECTED_KEY: 1.0, 'Loop/Highest_hypothesis_id/': 958.0,
        'Loop/Visual_inliers/': 162.0, 'Loop/Optimization_max_ang_error_ratio/': 35.1})
    assert update.events == ['rejected hypothesis 958 (162 inliers, graph error ratio 35.1)']


def test_island_reported_when_gate_off():
    tracker = SessionTracker()
    tracker.gate = False
    tracker.set_mode(True)
    tracker.update(100, 0, 0, {})
    assert tracker.describe().startswith('ISLAND')


def test_no_data_after_silence_and_fresh_fixes_after():
    tracker = SessionTracker(localized_after=2, lost_after_m=3.0, stale_after=10.0)
    tracker.set_mode(False)
    tracker.update(10, 5, 0, {}, now=100.0)
    tracker.update(11, 6, 0, {}, now=101.0)
    assert tracker.state == State.LOCALIZED
    assert not tracker.check_stale(now=110.0)          # 9 s: a slow update, not silence
    assert tracker.check_stale(now=112.0)              # 11 s: silence
    assert tracker.state == State.NO_DATA
    assert not tracker.check_stale(now=120.0)          # reported once
    update = tracker.update(12, 7, 0, {}, now=121.0)
    # Back, but one fix after the gap is not enough: the pose went unchecked meanwhile.
    assert update.state == State.NOT_LOCALIZED and 'RTAB-Map updates resumed' in update.events
    assert tracker.update(13, 8, 0, {}, now=122.0).state == State.LOCALIZED


def test_no_data_keeps_the_mapping_link():
    tracker = SessionTracker(stale_after=10.0)
    tracker.set_mode(True)
    tracker.update(100, 0, 0, {}, now=0.0)
    tracker.update(101, 0, 40, {}, now=1.0)
    assert tracker.state == State.MAPPING_LINKED
    tracker.check_stale(now=20.0)
    assert tracker.state == State.NO_DATA
    assert tracker.update(102, 0, 0, {}, now=21.0).state == State.MAPPING_LINKED


def test_no_data_needs_a_first_update_and_can_be_disabled():
    tracker = SessionTracker(stale_after=10.0)
    tracker.set_mode(False)
    assert not tracker.check_stale(now=1000.0)         # never had an update: still waiting
    off = SessionTracker(stale_after=0.0)
    off.set_mode(False)
    off.update(10, 0, 0, {}, now=0.0)
    assert not off.check_stale(now=1000.0)


def test_process_exit_is_reported_at_once():
    tracker = SessionTracker(stale_after=60.0)
    tracker.set_mode(True)
    assert not tracker.set_node_alive(False)           # never seen yet: a slow start, not a crash
    tracker.set_node_alive(True)
    tracker.update(100, 0, 40, {}, now=0.0)
    assert tracker.state == State.MAPPING_LINKED
    assert tracker.set_node_alive(False)               # gone: no need to wait 60 s
    assert tracker.state == State.NO_DATA
    assert tracker.describe().startswith('RTAB-Map process has exited')
    tracker.set_node_alive(True)                       # relaunched, nothing processed yet
    tracker.update(101, 0, 0, {}, now=5.0)
    assert tracker.state == State.MAPPING_LINKED


def test_silent_but_running_is_described_as_no_update():
    tracker = SessionTracker(stale_after=10.0)
    tracker.set_mode(False)
    tracker.set_node_alive(True)
    tracker.update(10, 0, 0, {}, now=0.0)
    tracker.check_stale(now=20.0)
    assert tracker.state == State.NO_DATA and tracker.describe().startswith('no RTAB-Map update')
