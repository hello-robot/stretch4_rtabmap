import math

import numpy as np
import pytest

pytest.importorskip('cv2')

from stretch4_rtabmap.navigation.nav2_diagnose import (  # noqa: E402
    describe_error,
    error_names,
    footprint_kernel,
    inside_polygon,
    map_cells_under,
    points_inside,
    scan_points,
    step_out_target,
)

RES = 0.05
# Test shapes only: a 16-gon base like stretch_core's robot_footprint.yaml, and one with the arm.
_A = 2.0 * math.pi * np.arange(16) / 16
BASE = np.column_stack((0.225 * np.cos(_A), 0.225 * np.sin(_A)))
# Base plus the arm stretched 0.8 m forward along +x, 0.1 m wide (what robot_footprint_publisher
# publishes with the arm out).
ARM_OUT = np.array([[0.225, -0.05], [0.8, -0.05], [0.8, 0.05], [0.225, 0.05],
                    [0.0, 0.225], [-0.225, 0.0], [0.0, -0.225]])


def test_error_names_cover_planner_and_controller():
    names = error_names()
    assert names[205] == 'START_OCCUPIED'
    assert names[208] == 'NO_VALID_PATH'
    assert names[105] == 'FAILED_TO_MAKE_PROGRESS'
    assert describe_error(205, 'Start occupied', names) == 'START_OCCUPIED (205): Start occupied'
    assert describe_error(999, '', names) == 'error (999)'


def test_inside_polygon():
    square = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])
    inside = inside_polygon([[0.5, 0.5], [1.5, 0.5], [0.5, -0.1]], square)
    assert inside.tolist() == [True, False, False]


def test_scan_points_are_moved_into_the_base_frame():
    # One beam straight ahead of a laser mounted 0.1 m forward and turned 90 deg left.
    pts = scan_points([1.0, float('inf')], 0.0, 0.1, 0.05, 10.0, (0.1, 0.0, math.pi / 2))
    assert pts.shape == (1, 2)
    assert np.allclose(pts[0], [0.1, 1.0])


def test_points_inside_follow_the_arm():
    pts = np.array([[0.1, 0.0], [0.6, 0.0], [0.6, 0.3]])
    assert points_inside(pts, BASE) == 1          # base circle: only the first
    assert points_inside(pts, ARM_OUT) == 2       # arm out: the one at the gripper too


def test_map_cells_under_uses_the_footprint_and_heading():
    grid = np.zeros((80, 80), dtype=np.int16)
    grid[40, 52] = 100                            # 0.6 m in +x of the robot cell (40, 40)
    pose = ((40 + 0.5) * RES, (40 + 0.5) * RES, 0.0)
    assert map_cells_under(grid, RES, (0.0, 0.0), pose, BASE) == 0
    assert map_cells_under(grid, RES, (0.0, 0.0), pose, ARM_OUT) == 1
    # Turned 90 deg, the arm points along +y and misses the cell.
    assert map_cells_under(grid, RES, (0.0, 0.0), (pose[0], pose[1], math.pi / 2), ARM_OUT) == 0


def test_kernel_is_centred_on_the_robot_cell():
    k = footprint_kernel(ARM_OUT, 0.0, RES)
    h = k.shape[0] // 2
    assert k.shape[0] == k.shape[1] and k.shape[0] % 2 == 1
    assert k[h, h] == 1 and k[h, h + 14] == 1 and k[h, h - 14] == 0   # arm reaches +x only


def test_step_out_target_leaves_a_wall_the_robot_overlaps():
    grid = np.zeros((60, 60), dtype=np.int16)
    grid[:, 30] = 100                             # wall cells centred on x = 1.525 m
    pose = (1.45, 1.5, 0.0)                       # base overlaps the wall
    tx, ty = step_out_target(grid, RES, (0.0, 0.0), pose, BASE, max_distance=0.5)
    assert tx < 1.45 and abs(ty - 1.5) < 0.05     # straight back, away from it
    assert map_cells_under(grid, RES, (0.0, 0.0), (tx, ty, 0.0), BASE) == 0
    assert step_out_target(grid, RES, (0.0, 0.0), pose, BASE, max_distance=0.1) is None


def test_step_out_target_accounts_for_the_arm():
    grid = np.zeros((80, 80), dtype=np.int16)
    grid[:, 50] = 100                             # wall at x = 2.525 m
    pose = (2.0, 2.0, 0.0)                        # base is 0.5 m clear, the arm reaches 2.8
    assert map_cells_under(grid, RES, (0.0, 0.0), pose, BASE) == 0
    assert map_cells_under(grid, RES, (0.0, 0.0), pose, ARM_OUT) > 0
    tx, ty = step_out_target(grid, RES, (0.0, 0.0), pose, ARM_OUT, max_distance=0.6)
    assert tx < 2.0 - 0.25                        # backs off until the gripper clears the wall
    assert map_cells_under(grid, RES, (0.0, 0.0), (tx, ty, 0.0), ARM_OUT) == 0
