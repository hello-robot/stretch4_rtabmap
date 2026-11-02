"""
Why nav2 will not go somewhere: error names, and what the robot really has around it.

Used by mapped_navigator's UNSTUCK state to judge "is the robot really in an obstacle?".

The planner answers START_OCCUPIED when the global costmap has an obstacle under the robot. That
costmap is static_layer (RTAB-Map's /map) + obstacle_layer (the scan) + inflation, so the obstacle
is either in the map (stale cells, or the pose is a little off), or old scan marks, or just the
inflation of a wall the robot stands next to. The live scan says which of those is real.

"""

import math

import cv2
import numpy as np

from stretch4_rtabmap.navigation.frontiers import _nearest_cell

LETHAL = 254          # nav2_costmap_2d::LETHAL_OBSTACLE
INSCRIBED = 253       # INSCRIBED_INFLATED_OBSTACLE: the footprint touches an obstacle


def error_names():
    """Return {code: NAME} for the nav2 planner and controller error codes."""
    from nav2_msgs.action import ComputePathToPose, FollowPath
    names = {}
    for result in (ComputePathToPose.Result, FollowPath.Result):
        for name in dir(result):
            value = getattr(result, name)
            if name.isupper() and isinstance(value, int) and name != 'NONE':
                names[value] = name
    return names


def describe_error(code, message='', names=None):
    """Return 'START_OCCUPIED (205): Start occupied' style text for a nav2 result."""
    names = names if names is not None else error_names()
    text = f'{names.get(code, "error")} ({code})'
    return f'{text}: {message}' if message else text


def transform_points(points, pose):
    """Move (N, 2) points by pose (x, y, yaw): from that pose's frame into its parent."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    return np.column_stack((x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]))


def inside_polygon(points, polygon):
    """Return a bool array: which (N, 2) points are inside the (K, 2) polygon (even-odd rule)."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    poly = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
    if len(pts) == 0 or len(poly) < 3:
        return np.zeros(len(pts), dtype=bool)
    x, y = pts[:, 0:1], pts[:, 1:2]
    x0, y0 = poly[:, 0], poly[:, 1]
    x1, y1 = np.roll(x0, -1), np.roll(y0, -1)
    crosses = (y0 > y) != (y1 > y)
    with np.errstate(divide='ignore', invalid='ignore'):
        x_at = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
    return np.count_nonzero(crosses & (x < x_at), axis=1) % 2 == 1


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, laser_in_base):
    """Return the valid scan points (N, 2) in the base frame; laser_in_base is the laser pose."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_increment * np.arange(r.size)
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return transform_points(np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]))),
                            laser_in_base)


def points_inside(points, footprint):
    """Count the base-frame points inside the base-frame footprint polygon."""
    return int(np.count_nonzero(inside_polygon(points, footprint)))


def map_cells_under(grid, resolution, origin_xy, pose, footprint, occupied_threshold=50):
    """Count the occupied /map cells whose centre is under the footprint at pose (x, y, yaw)."""
    grid = np.asarray(grid)
    h, w = grid.shape
    poly = transform_points(footprint, pose)
    lo, hi = poly.min(axis=0), poly.max(axis=0)
    c0 = max(int((lo[0] - origin_xy[0]) / resolution), 0)
    r0 = max(int((lo[1] - origin_xy[1]) / resolution), 0)
    c1 = min(int((hi[0] - origin_xy[0]) / resolution) + 1, w)
    r1 = min(int((hi[1] - origin_xy[1]) / resolution) + 1, h)
    if c0 >= c1 or r0 >= r1:
        return 0
    rows, cols = np.mgrid[r0:r1, c0:c1]
    centres = np.column_stack((origin_xy[0] + (cols.ravel() + 0.5) * resolution,
                               origin_xy[1] + (rows.ravel() + 0.5) * resolution))
    under = inside_polygon(centres, poly).reshape(rows.shape)
    return int(np.count_nonzero(under & (grid[r0:r1, c0:c1] >= occupied_threshold)))


def footprint_kernel(footprint, yaw, resolution, margin=0.0):
    """
    Rasterise the footprint, turned by yaw and grown by margin, into an odd square kernel.

    Element (h + dy, h + dx) is set when the footprint covers the cell dx, dy cells from the
    robot's; the centre element is the robot's own cell.
    """
    poly = transform_points(footprint, (0.0, 0.0, yaw))
    reach = np.hypot(poly[:, 0], poly[:, 1]).max() + margin
    h = int(math.ceil(reach / resolution)) + 1
    offs = np.arange(-h, h + 1) * resolution
    dx, dy = np.meshgrid(offs, offs)
    centres = np.column_stack((dx.ravel(), dy.ravel()))
    covered = inside_polygon(centres, poly)
    if margin > 0.0:
        # Within `margin` of an edge also counts.
        a, b = poly, np.roll(poly, -1, axis=0)
        ab = b - a
        rel = centres[:, None, :] - a
        t = np.clip((rel * ab).sum(-1) / (ab * ab).sum(-1), 0.0, 1.0)
        gap = rel - t[..., None] * ab
        covered |= np.hypot(gap[..., 0], gap[..., 1]).min(axis=1) <= margin
    return covered.reshape(dx.shape).astype(np.uint8)


def step_out_target(grid, resolution, origin_xy, pose, footprint, max_distance, margin=0.03,
                    occupied_threshold=50):
    """
    Return the nearest (x, y) where the footprint, heading kept, clears every occupied /map cell.

    The footprint is grown by `margin`. Found by dilating the occupied cells with the footprint
    shape: what is left are the robot positions where no part of the footprint, arm included,
    overlaps the map's obstacles. None if there is none within max_distance.
    """
    grid = np.asarray(grid)
    occupied = (grid >= occupied_threshold).astype(np.uint8)
    free = (grid >= 0) & (grid < occupied_threshold)
    kernel = footprint_kernel(footprint, pose[2], resolution, margin)
    # cv2.dilate sets a cell when src at (cell + kernel offset) is set for any kernel element:
    # exactly "the footprint, with the robot on this cell, covers an occupied cell".
    collides = cv2.dilate(occupied, kernel, borderType=cv2.BORDER_CONSTANT, borderValue=0) > 0
    cell = _nearest_cell(free & ~collides, resolution, origin_xy, pose[:2], max_m=max_distance)
    if cell is None:
        return None
    row, col = cell
    return origin_xy[0] + (col + 0.5) * resolution, origin_xy[1] + (row + 0.5) * resolution
