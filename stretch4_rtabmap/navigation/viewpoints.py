"""
Pick places near a goal from which the robot can look at it and update the map.

A viewpoint is a free cell with room for the robot, in the free space CONNECTED to the robot,
as close to the goal as that space gets.

"""

from dataclasses import dataclass
import math

import cv2
import numpy as np

UNKNOWN = -1


@dataclass(frozen=True)
class Viewpoint:
    x: float
    y: float
    yaw: float          # facing the goal
    distance: float     # to the goal, metres
    frontier: bool      # borders unknown space


def find_viewpoints(grid, resolution, origin_xy, goal_xy, radius, clearance, robot_xy=None,
                    spacing=0.75, exclude=(), exclude_radius=1.0, max_results=20,
                    occupied_threshold=50):
    """
    Return viewpoints, best first.

    grid          (H, W) int array, OccupancyGrid values.
    origin_xy     map coordinates of cell (0, 0)'s corner.
    radius        how far from the goal to look; if no usable cell is that close, the nearest
                  ones are kept anyway.
    clearance     minimum distance from any occupied cell, metres.
    robot_xy      keep only cells connected to the robot through free, clear cells. None
                  skips that test.
    spacing       minimum distance between two viewpoints in the list.
    exclude       (x, y) points already tried; no viewpoint within exclude_radius of them.
    max_results   length of the list at most.
    """
    grid = np.asarray(grid)
    free = (grid >= 0) & (grid < occupied_threshold)
    occupied = grid >= occupied_threshold
    if not free.any():
        return []

    # Metres from each cell to the nearest occupied cell (unknown is not counted as occupied: it
    # is the planner's job to refuse unknown space, not ours to guess what is in it).
    clear_m = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5) * resolution
    usable = free & (clear_m >= clearance)

    if robot_xy is not None:
        usable = _connected_to(usable, resolution, origin_xy, robot_xy)

    unknown = grid == UNKNOWN
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]

    rows, cols = np.nonzero(usable)
    if rows.size == 0:
        return []
    xs = origin_xy[0] + (cols + 0.5) * resolution
    ys = origin_xy[1] + (rows + 0.5) * resolution
    dist = np.hypot(xs - goal_xy[0], ys - goal_xy[1])
    keep = dist <= max(radius, dist.min() + spacing)
    for ex, ey in exclude:
        keep &= np.hypot(xs - ex, ys - ey) > exclude_radius
    rows, cols, xs, ys, dist = rows[keep], cols[keep], xs[keep], ys[keep], dist[keep]
    frontier = near_unknown[rows, cols]

    # Nearest the goal first, frontier first on a tie, then by position so that equal candidates
    # still come out in a fixed order; skip anything within `spacing` of one already picked.
    picked = []
    for i in np.lexsort((xs, ys, ~frontier, np.round(dist, 3))):
        if len(picked) >= max_results:
            break
        if not picked or np.hypot(xs[picked] - xs[i], ys[picked] - ys[i]).min() >= spacing:
            picked.append(i)
    return [Viewpoint(float(xs[i]), float(ys[i]),
                      math.atan2(goal_xy[1] - ys[i], goal_xy[0] - xs[i]),
                      float(dist[i]), bool(frontier[i]))
            for i in picked]


def _connected_to(usable, resolution, origin_xy, robot_xy):
    """Keep the part of `usable` connected to the robot; empty if the robot is off it."""
    _, labels = cv2.connectedComponents(usable.astype(np.uint8), connectivity=8)
    col = int((robot_xy[0] - origin_xy[0]) / resolution)
    row = int((robot_xy[1] - origin_xy[1]) / resolution)
    # The robot's own cell can fail the clearance test next to a wall; take the nearest usable
    # cell within a metre instead.
    rows, cols = np.nonzero(labels > 0)
    if rows.size == 0:
        return np.zeros_like(usable)
    d = np.hypot(rows - row, cols - col)
    nearest = int(np.argmin(d))
    if d[nearest] * resolution > 1.0:
        return np.zeros_like(usable)
    return labels == labels[rows[nearest], cols[nearest]]
