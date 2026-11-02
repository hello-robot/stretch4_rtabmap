import math

import numpy as np
import pytest

pytest.importorskip('cv2')

from stretch4_rtabmap.navigation.frontiers import find_frontiers  # noqa: E402

RES = 0.05
ORIGIN = (0.0, 0.0)


def room(width_m=4.0, height_m=3.0):
    """Return a walled room of free space, everything outside unknown."""
    w, h = int(width_m / RES), int(height_m / RES)
    grid = np.full((h + 40, w + 40), -1, dtype=np.int16)
    grid[20:20 + h, 20:20 + w] = 0
    grid[20:20 + h, 20] = grid[20:20 + h, 19 + w] = 100
    grid[20, 20:20 + w] = grid[19 + h, 20:20 + w] = 100
    return grid


def xy(row, col):
    return ORIGIN[0] + (col + 0.5) * RES, ORIGIN[1] + (row + 0.5) * RES


def test_closed_room_has_no_frontiers():
    grid = room()
    assert find_frontiers(grid, RES, ORIGIN, xy(50, 60)) == []


def test_door_gives_one_frontier_with_a_reachable_target_facing_it():
    grid = room()
    grid[40:60, 99] = 0                      # 1 m wide gap in the right wall, unknown beyond
    frontiers = find_frontiers(grid, RES, ORIGIN, xy(50, 60))
    assert len(frontiers) == 1
    f = frontiers[0]
    assert f.size == 20 and math.isclose(f.size_m, 1.0)
    tx, ty, yaw = f.target
    # Target is inside the room, clear of the walls, and faces the door (+x).
    assert grid[int(ty / RES), int(tx / RES)] == 0
    assert abs(yaw) < 0.3
    assert f.middle[0] > tx


def test_frontier_behind_a_wall_is_not_reachable():
    grid = room()
    grid[:, 105:115] = 0                     # free strip outside the right wall, unknown around it
    assert find_frontiers(grid, RES, ORIGIN, xy(50, 60)) == []


def test_small_frontiers_are_dropped():
    grid = room()
    grid[50:55, 99] = 0                      # 0.25 m gap
    assert find_frontiers(grid, RES, ORIGIN, xy(50, 60), min_frontier_size=0.5) == []
    assert len(find_frontiers(grid, RES, ORIGIN, xy(50, 60), min_frontier_size=0.2)) == 1


def test_goal_scale_prefers_the_frontier_toward_the_goal():
    grid = room()
    grid[40:60, 99] = 0                      # right door, 1 m
    grid[25:45, 20] = 0                      # left door, 1 m
    robot = xy(50, 60)
    goal = (6.0, 2.5)                        # well to the right
    plain = find_frontiers(grid, RES, ORIGIN, robot)
    biased = find_frontiers(grid, RES, ORIGIN, robot, goal_xy=goal, goal_scale=2.0)
    assert len(plain) == len(biased) == 2
    assert biased[0].middle[0] > 4.0         # the right-hand door first
    assert biased[0].goal_distance < biased[1].goal_distance


def test_bigger_frontier_wins_at_equal_distance():
    grid = room()
    grid[40:60, 99] = 0                      # 1 m on the right
    grid[30:70, 20] = 0                      # 2 m on the left
    robot = xy(50, 60)                       # 2 m from both walls
    frontiers = find_frontiers(grid, RES, ORIGIN, robot)
    assert frontiers[0].size == 40


def test_exclude_drops_tried_targets_and_max_goal_distance_limits_the_search():
    grid = room()
    grid[40:60, 99] = 0
    first = find_frontiers(grid, RES, ORIGIN, xy(50, 60))[0]
    assert find_frontiers(grid, RES, ORIGIN, xy(50, 60), exclude=[first.target[:2]]) == []
    assert find_frontiers(grid, RES, ORIGIN, xy(50, 60), goal_xy=(0.2, 0.2),
                          max_goal_distance=1.0) == []


def test_deterministic():
    rng = np.random.default_rng(0)
    grid = room(6.0, 5.0)
    grid[rng.random(grid.shape) < 0.02] = -1   # speckle of unknown cells inside the room
    a = find_frontiers(grid, RES, ORIGIN, xy(60, 80), min_frontier_size=0.0)
    b = find_frontiers(grid, RES, ORIGIN, xy(60, 80), min_frontier_size=0.0)
    assert a == b and len(a) > 5


def test_robot_off_the_free_space():
    grid = np.full((50, 50), -1, dtype=np.int16)
    assert find_frontiers(grid, RES, ORIGIN, (1.0, 1.0)) == []
