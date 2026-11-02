import numpy as np

from stretch4_rtabmap.navigation.viewpoints import find_viewpoints

RES = 0.05


def room(width_m=6.0, height_m=4.0):
    """Free room with a one-cell wall all around, origin at (0, 0)."""
    grid = np.zeros((int(height_m / RES), int(width_m / RES)), dtype=np.int16)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 100
    return grid


def test_nearest_free_cell_to_blocked_goal_first():
    grid = room()
    # A box around the goal at (3, 2), as if furniture had been moved onto it.
    grid[30:50, 50:70] = 100
    found = find_viewpoints(grid, RES, (0.0, 0.0), (3.0, 2.0), radius=3.0, clearance=0.3)
    assert found
    best = found[0]
    # Clear of the box by the clearance (measured to cell centres), and facing the goal.
    gap = np.hypot(max(2.5 - best.x, 0.0, best.x - 3.5), max(1.5 - best.y, 0.0, best.y - 2.5))
    assert gap >= 0.3 - RES
    assert np.isclose(np.arctan2(2.0 - best.y, 3.0 - best.x), best.yaw)
    rounded = [round(v.distance, 3) for v in found]
    assert rounded == sorted(rounded)


def test_frontier_preferred_on_equal_distance():
    grid = room()
    grid[:, 90:] = -1      # unknown east half beyond x = 4.5
    found = find_viewpoints(grid, RES, (0.0, 0.0), (5.5, 2.0), radius=3.0, clearance=0.2)
    assert found[0].frontier


def test_clearance_and_exclusion():
    grid = room()
    found = find_viewpoints(grid, RES, (0.0, 0.0), (3.0, 2.0), radius=5.0, clearance=0.5)
    assert all(0.5 <= v.x <= 5.5 and 0.5 <= v.y <= 3.5 for v in found)
    again = find_viewpoints(grid, RES, (0.0, 0.0), (3.0, 2.0), radius=5.0, clearance=0.5,
                            exclude=[(3.0, 2.0)], exclude_radius=1.0)
    assert all(np.hypot(v.x - 3.0, v.y - 2.0) > 1.0 for v in again)


def test_blocked_doorway_picks_reachable_side():
    grid = room(8.0, 4.0)
    grid[:, 80] = 100           # wall at x = 4.0
    grid[30:50, 80] = 100       # its doorway, y 1.5 to 2.5, closed by moved furniture
    goal, robot = (6.0, 2.0), (1.0, 2.0)
    found = find_viewpoints(grid, RES, (0.0, 0.0), goal, radius=3.0, clearance=0.3,
                            robot_xy=robot)
    assert found
    # Every viewpoint is on the robot's side of the wall, the first one right at the doorway.
    assert all(v.x < 4.0 for v in found)
    assert found[0].x > 3.5 and 1.5 <= found[0].y <= 2.5
    assert all(np.hypot(a.x - b.x, a.y - b.y) >= 0.75
               for i, a in enumerate(found) for b in found[i + 1:])


def test_robot_off_free_space_gives_nothing():
    grid = room()
    assert find_viewpoints(grid, RES, (0.0, 0.0), (3.0, 2.0), radius=3.0, clearance=0.3,
                           robot_xy=(30.0, 30.0)) == []


def test_deterministic():
    grid = room()
    grid[20:60, 60] = 100
    args = (grid, RES, (0.0, 0.0), (4.0, 2.0))
    assert find_viewpoints(*args, radius=4.0, clearance=0.3) == \
        find_viewpoints(*args, radius=4.0, clearance=0.3)


def test_no_free_space():
    grid = np.full((10, 10), -1, dtype=np.int16)
    assert find_viewpoints(grid, RES, (0.0, 0.0), (0.2, 0.2), radius=1.0, clearance=0.1) == []
