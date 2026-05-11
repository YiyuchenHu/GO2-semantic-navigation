#!/usr/bin/env python3
"""Day 8 gate #1 — offline unit test for the frontier-detection algorithm.

Imports `compute_frontier_clusters` from go2_navigation.frontier_explorer_node
and runs it on a hand-built 50x50 OccupancyGrid:

    columns  0 .. 24  →  known free   (value 0)
    columns 25 .. 49  →  unknown      (value -1)

The frontier should therefore be a vertical strip at column 24 (the
last free column, which has unknown 4-neighbours at column 25).

Pass criteria:
  * compute_frontier_clusters returns success=True
  * top_clusters non-empty
  * every returned centroid has cell-x within ±5 cells of column 24
  * all returned scores are finite numbers, sorted descending

This test does NOT need rclpy or Nav2 — it links directly to the
algorithm function. The on-the-wire service is exercised by gate #2
(consumption) where a live sim is required anyway.
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import Tuple


# ---------------------------------------------------------------------
# Lightweight stand-ins for nav_msgs/MapMetaData + geometry_msgs/Point.
# We intentionally avoid `import nav_msgs.msg` so this script can run
# without sourcing the ROS workspace — only `numpy`, `cv2` and our own
# package need to be importable.
# ---------------------------------------------------------------------


@dataclass
class _Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class _Pose:
    position: _Point


@dataclass
class _MapInfo:
    width: int
    height: int
    resolution: float
    origin: _Pose


def _build_test_grid() -> Tuple["np.ndarray", _MapInfo]:
    """50x50 grid: left half free, right half unknown."""
    import numpy as np

    grid = np.zeros((50, 50), dtype=np.int16)
    grid[:, 25:] = -1  # right half unknown
    info = _MapInfo(
        width=50,
        height=50,
        resolution=0.10,  # 10 cm cells → 5 m × 5 m world
        origin=_Pose(position=_Point(x=0.0, y=0.0, z=0.0)),
    )
    return grid, info


def _world_x_to_cell(wx: float, info: _MapInfo) -> float:
    return (wx - info.origin.position.x) / info.resolution - 0.5


def main() -> int:
    try:
        import numpy as np  # noqa: F401  (build_test_grid uses it)
        import cv2  # noqa: F401  (compute_frontier_clusters needs it)
    except Exception as exc:
        print(f"ERROR_IMPORT_DEPS: {type(exc).__name__}: {exc}")
        return 2

    try:
        from go2_navigation.frontier_explorer_node import (
            compute_frontier_clusters,
        )
    except Exception as exc:
        print(
            f"ERROR_IMPORT_PACKAGE: {type(exc).__name__}: {exc}; "
            "have you sourced install/setup.bash after `colcon build "
            "--packages-select go2_navigation`?"
        )
        return 2

    grid, info = _build_test_grid()

    # Place the "robot" at the world origin (which is the corner of
    # cell (0,0) → world (0.05, 0.05) cell-centre). Distance scoring
    # therefore prefers the closer end of the frontier strip — fine
    # for our centroid-x assertion since all centroids share x.
    robot_xy = (0.0, 0.0)

    success, message, top = compute_frontier_clusters(
        grid,
        info,
        robot_xy,
        min_cluster_size=5,        # one strip, 50 cells; safely above 5.
        info_gain_radius_m=1.0,    # ~10 cells radius — info_gain > 0.
        distance_weight=1.0,
        max_frontiers=5,
    )

    if not success:
        print(f"ERROR_NOT_SUCCESS message={message!r}")
        return 1
    if not top:
        print(f"ERROR_EMPTY message={message!r}")
        return 1

    # Sanity: scores finite + descending.
    prev_score = math.inf
    for i, (wx, wy, ig, d, score, sz) in enumerate(top):
        if not math.isfinite(score):
            print(f"ERROR_NONFINITE i={i} score={score}")
            return 1
        if score > prev_score + 1e-6:
            print(
                f"ERROR_NOT_SORTED i={i} score={score} prev={prev_score}"
            )
            return 1
        prev_score = score

    # Centroid-x check: column 24 ± 5.
    centroid_xs_world = [c[0] for c in top]
    centroid_xs_cell = [_world_x_to_cell(wx, info) for wx in centroid_xs_world]
    bad = [
        (i, cx) for i, cx in enumerate(centroid_xs_cell) if abs(cx - 24.0) > 5.0
    ]
    if bad:
        bad_str = ", ".join(f"#{i}:cx={cx:.1f}" for i, cx in bad)
        print(
            f"ERROR_BAD_CENTROIDS expected_cell_x≈24 (±5), got: {bad_str}"
        )
        return 1

    # Information gain on the best cluster must be > 0 — the centroid
    # sits one cell away from the unknown half, so the radius-bounded
    # box should always overlap unknown cells.
    if top[0][2] <= 0:
        print(f"ERROR_ZERO_INFO_GAIN top0={top[0]}")
        return 1

    summary = (
        f"frontiers={len(top)} best_centroid_world=({top[0][0]:.3f},"
        f"{top[0][1]:.3f}) best_centroid_cell_x="
        f"{centroid_xs_cell[0]:.1f} best_info_gain={top[0][2]} "
        f"best_distance={top[0][3]:.3f}m best_score={top[0][4]:.2f} "
        f"all_centroid_cell_x=[{','.join(f'{x:.1f}' for x in centroid_xs_cell)}] "
        f"message={message!r} pass=1"
    )
    print(summary)
    return 0


if __name__ == "__main__":
    sys.exit(main())
