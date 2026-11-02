"""
Frontier search on an occupancy grid.

* The reachable space is the 4-connected component of free cells containing the robot's nearest
  free cell (cv2.connectedComponents).
* Frontiers are 8-connected groups of unknown cells with a free 4-neighbour; a group is kept if
  any of its cells touches the reached space.
* Per frontier: size in cells, centroid, and `middle`, the cell closest to the robot, with that
  distance as `min_distance`. Frontiers shorter than `min_frontier_size` metres are dropped.
* Cost, in metres: potential_scale * distance - gain_scale * size + goal_scale * distance to the
  goal, lowest first, so a robot that cannot reach a goal explores the frontiers on the way to it
  first instead of the largest one in the building. Targets already tried are passed in
  `exclude` and skipped.
* A frontier's centroid is often in unknown space or inside a wall, and Nav2 here plans with
  allow_unknown: false. Each frontier gets a `target` instead: the reachable free cell with room
  for the robot that is nearest the frontier (nearest its cell closest to the goal when there is
  a goal), facing the frontier, so the head cameras look into the unknown area.
* Ties are broken by position, so the same grid always gives the same list in the same order.
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np

UNKNOWN = -1


@dataclass(frozen=True)
class Frontier:
    size: int                   # cells
    size_m: float               # size * resolution, compared with min_frontier_size
    centroid: tuple             # (x, y) mean of the frontier cells
    middle: tuple               # (x, y) frontier cell closest to the robot
    min_distance: float         # robot to `middle`, metres
    goal_distance: float        # goal to the frontier's nearest cell, metres; 0 without a goal
    cost: float
    target: tuple               # (x, y, yaw) to drive to, or None if no reachable cell has room


def find_frontiers(grid, resolution, origin_xy, robot_xy, goal_xy=None, min_frontier_size=0.5,
                   potential_scale=1.0, gain_scale=1.0, goal_scale=0.0, clearance=0.25,
                   max_goal_distance=None, exclude=(), exclude_radius=0.25,
                   occupied_threshold=50):
    """
    Return the frontiers reachable from the robot, lowest cost first.

    grid               (H, W) int array, OccupancyGrid values.
    origin_xy          map coordinates of cell (0, 0)'s corner.
    goal_xy            optional goal; with goal_scale > 0 frontiers nearer it cost less.
    clearance          metres from any occupied cell a target must be.
    max_goal_distance  drop frontiers farther than this from the goal (metres).
    exclude            (x, y) targets already tried; frontiers whose target is within
                       exclude_radius of one are dropped.
    """
    grid = np.asarray(grid)
    free = (grid >= 0) & (grid < occupied_threshold)
    occupied = grid >= occupied_threshold
    unknown = grid == UNKNOWN

    start = _nearest_cell(free, resolution, origin_xy, robot_xy)
    if start is None:
        return []
    _, free_labels = cv2.connectedComponents(free.astype(np.uint8), connectivity=4)
    reached = free_labels == free_labels[start]

    has_free_nbr = _shift_any(free)
    candidate = unknown & has_free_nbr            # unknown cells next to free space
    seeds = candidate & _shift_any(reached)        # the ones the search actually runs into
    if not seeds.any():
        return []
    n, labels = cv2.connectedComponents(candidate.astype(np.uint8), connectivity=8)
    keep_label = np.zeros(n, dtype=bool)
    keep_label[np.unique(labels[seeds])] = True
    keep_label[0] = False

    rows, cols = np.nonzero(keep_label[labels])
    lab = labels[rows, cols]
    xs = origin_xy[0] + (cols + 0.5) * resolution
    ys = origin_xy[1] + (rows + 0.5) * resolution
    robot_d = np.hypot(xs - robot_xy[0], ys - robot_xy[1])
    goal_d = (np.hypot(xs - goal_xy[0], ys - goal_xy[1]) if goal_xy is not None
              else np.zeros_like(xs))

    # Nearest cell with room for the robot, in the reached space, for every cell of the grid.
    clear_m = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5) * resolution
    usable = reached & (clear_m >= clearance)
    nearest_usable = _nearest_index_map(usable)

    frontiers = []
    for label in np.unique(lab):
        idx = np.nonzero(lab == label)[0]
        size = idx.size
        if size * resolution < min_frontier_size:
            continue
        # Lowest distance, then lowest y, then lowest x: a fixed choice among equal cells.
        m = idx[np.lexsort((xs[idx], ys[idx], np.round(robot_d[idx], 6)))[0]]
        g = idx[np.lexsort((xs[idx], ys[idx], np.round(goal_d[idx], 6)))[0]]
        if max_goal_distance is not None and goal_xy is not None \
                and goal_d[g] > max_goal_distance:
            continue
        anchor = g if goal_xy is not None else m
        target = None
        if nearest_usable is not None:
            tr, tc = nearest_usable[0][rows[anchor], cols[anchor]], \
                nearest_usable[1][rows[anchor], cols[anchor]]
            if tr >= 0:
                tx = origin_xy[0] + (tc + 0.5) * resolution
                ty = origin_xy[1] + (tr + 0.5) * resolution
                target = (float(tx), float(ty),
                          math.atan2(ys[anchor] - ty, xs[anchor] - tx))
        if target is not None and any(math.hypot(target[0] - ex, target[1] - ey) < exclude_radius
                                      for ex, ey in exclude):
            continue
        size_m = size * resolution
        cost = potential_scale * robot_d[m] - gain_scale * size_m + goal_scale * goal_d[g]
        frontiers.append(Frontier(
            size=int(size), size_m=float(size_m),
            centroid=(float(xs[idx].mean()), float(ys[idx].mean())),
            middle=(float(xs[m]), float(ys[m])), min_distance=float(robot_d[m]),
            goal_distance=float(goal_d[g]) if goal_xy is not None else 0.0,
            cost=float(cost), target=target))

    frontiers.sort(key=lambda f: (round(f.cost, 6), f.middle[1], f.middle[0]))
    return frontiers


def _shift_any(mask):
    """Return True where any 4-neighbour of the cell is True."""
    out = np.zeros_like(mask)
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    return out


def _nearest_cell(mask, resolution, origin_xy, xy, max_m=1.0):
    """(row, col) of the True cell nearest xy within max_m."""
    rows, cols = np.nonzero(mask)
    if rows.size == 0:
        return None
    col = (xy[0] - origin_xy[0]) / resolution - 0.5
    row = (xy[1] - origin_xy[1]) / resolution - 0.5
    d = np.hypot(rows - row, cols - col)
    order = np.lexsort((cols, rows, np.round(d, 6)))
    i = order[0]
    if d[i] * resolution > max_m:
        return None
    return int(rows[i]), int(cols[i])


def _nearest_index_map(mask):
    """(row map, col map) of the nearest True cell for every cell, or None if mask is empty."""
    if not mask.any():
        return None
    # distanceTransformWithLabels labels each zero pixel and gives every pixel the label of the
    # nearest one; the zeros here are the usable cells.
    _, label_map = cv2.distanceTransformWithLabels(
        (~mask).astype(np.uint8), cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL)
    zero_rows, zero_cols = np.nonzero(mask)
    # Labels are assigned to zero pixels in row-major scan order, starting at 1.
    lut_r = np.concatenate(([-1], zero_rows))
    lut_c = np.concatenate(([-1], zero_cols))
    return lut_r[label_map], lut_c[label_map]
