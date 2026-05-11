"""Day 8 — Frontier exploration goal provider.

Single responsibility: cache the latest /map (nav_msgs/OccupancyGrid),
and on demand (service /get_frontiers) return a ranked list of frontier
goal poses for the task_coordinator to drive Nav2 to.

A frontier cell is a *known free* cell (occupancy value 0) that has at
least one *unknown* (occupancy value -1) cell among its 8-connected
(3x3) neighbours. The 3x3 kernel — wider than the classic 4-neighbour
cross — picks up frontiers along diagonal map edges (corners, narrow
passages) that the cross would miss. This matches the TB3 frontier
node's frontier definition.

Frontier cells are clustered with 8-connected components
(cv2.connectedComponents). Each cluster's centroid is one candidate
goal; its score combines an information-gain proxy (count of unknown
cells in a small radius around the centroid) and the Euclidean distance
from the requesting robot pose:

    score = info_gain - distance_weight * distance_to_robot

Top ``max_frontiers`` clusters are returned, sorted by score descending.

Centroid safety
---------------
Each candidate centroid is validated against the navigation cost field
before being returned. Two sources are supported, in priority order:

1. **/global_costmap/costmap** (preferred). nav2's inflated costmap
   already encodes "occupied + robot_radius + inflation_radius", so a
   centroid with cost in [0, ``costmap_safe_max_cost``) is by
   definition a goal the global planner can accept. This mirrors the
   TB3 reference design (cost < 75 threshold).
2. **distanceTransform on /map** (fallback, used until the costmap
   topic latches). Treats only OccupancyGrid==100 as obstacle and
   requires the centroid to sit at least ``safety_radius_m`` metres
   away from any such cell. Unknown (-1) cells are intentionally NOT
   treated as obstacles — by definition a frontier centroid sits next
   to unknown space.

In both modes, if the raw cluster centroid fails the safety test, we
search the cluster bbox (and, failing that, a ``snap_search_radius_m``
window around the centroid) for the closest free + safe cell. If
nothing in that window passes, the entire cluster is dropped.

This node DELIBERATELY does not own a state machine, send any
NavigateToPose action, or know anything about semantic targets — it is
a stateless query node. The task_coordinator owns the EXPLORE state
and the action client; this node just answers "what's a useful place
to go look at?" when asked.

Visualization: every successful service call also publishes a
MarkerArray on /frontier_markers — one SPHERE per returned frontier,
coloured from green (best score) to red (worst). RViz operators can
toggle this display independently of the action goal markers.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from go2_msgs.srv import GetFrontiers
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


# ----------------------------------------------------------------------
# Pure-function algorithm core — extracted so check_day8 can unit-test
# the frontier detector without spinning up rclpy or a service client.
# ----------------------------------------------------------------------

# (world_x, world_y, info_gain, distance_to_robot, score, cluster_size)
FrontierCluster = Tuple[float, float, int, float, float, int]


def cell_to_world(cx: float, cy: float, info: Any) -> Tuple[float, float]:
    """OccupancyGrid origin is the corner of cell (0,0); +0.5 → cell centre."""
    wx = info.origin.position.x + (cx + 0.5) * info.resolution
    wy = info.origin.position.y + (cy + 0.5) * info.resolution
    return wx, wy


class _SafetyChecker:
    """Two-mode centroid-safety predicate, modelled after the TB3 design.

    Built once per service call from the latest /map (always required)
    and the latest /global_costmap/costmap (optional). Exposes two
    methods:

    * :meth:`is_safe(ix, iy)` — True iff cell (ix, iy) on the /map grid
      is acceptable as a Nav2 goal.
    * :meth:`describe()` — short string for log/response messages.

    When the costmap is available we use it as the primary signal:
    a /map cell (ix, iy) is converted to its world coordinate, then
    re-projected into the costmap's own grid. Cost in
    ``[0, costmap_safe_max_cost)`` is "safe" — this is exactly TB3's
    `cost < 75` rule. Cells outside the costmap bounds are treated as
    unsafe (we don't know if they're navigable).

    When the costmap is missing we fall back to a distance transform
    over occupied cells in /map, requiring the centroid to be at least
    ``safety_radius_m`` away from any occupied (>50) cell.
    """

    def __init__(
        self,
        grid: np.ndarray,
        info: Any,
        free_mask: np.ndarray,
        *,
        safety_radius_m: float,
        costmap_grid: Optional[np.ndarray],
        costmap_info: Optional[Any],
        costmap_safe_max_cost: int,
    ) -> None:
        self._info = info
        self._free_mask = free_mask
        self._h, self._w = grid.shape
        self._costmap_grid = costmap_grid
        self._costmap_info = costmap_info
        self._costmap_safe_max_cost = costmap_safe_max_cost

        if costmap_grid is not None and costmap_info is not None:
            self._mode = "costmap"
            self._safe_distance: Optional[np.ndarray] = None
            self._safety_cells = 0.0
        else:
            self._mode = "distance_transform"
            occupied_mask = (grid > 50).astype(np.uint8)
            if occupied_mask.any():
                non_obstacle = (1 - occupied_mask).astype(np.uint8)
                self._safe_distance = cv2.distanceTransform(
                    non_obstacle, distanceType=cv2.DIST_L2, maskSize=3
                )
            else:
                self._safe_distance = np.full(
                    grid.shape, 1e6, dtype=np.float32
                )
            self._safety_cells = max(
                0.0, safety_radius_m / info.resolution
            )

    @property
    def mode(self) -> str:
        return self._mode

    def describe(self) -> str:
        if self._mode == "costmap":
            return (
                f"costmap-cost<{self._costmap_safe_max_cost} "
                f"({self._costmap_info.width}x{self._costmap_info.height} "
                f"res={self._costmap_info.resolution:.3f}m)"
            )
        return (
            f"distance>={self._safety_cells:.1f}cells "
            f"(no /global_costmap latched yet)"
        )

    def is_safe(self, ix: int, iy: int) -> bool:
        if not (0 <= ix < self._w and 0 <= iy < self._h):
            return False
        if self._free_mask[iy, ix] != 1:
            return False
        if self._mode == "costmap":
            wx = (
                self._info.origin.position.x
                + (ix + 0.5) * self._info.resolution
            )
            wy = (
                self._info.origin.position.y
                + (iy + 0.5) * self._info.resolution
            )
            mx = int(
                (wx - self._costmap_info.origin.position.x)
                / self._costmap_info.resolution
            )
            my = int(
                (wy - self._costmap_info.origin.position.y)
                / self._costmap_info.resolution
            )
            if not (
                0 <= mx < self._costmap_info.width
                and 0 <= my < self._costmap_info.height
            ):
                return False
            cost = int(self._costmap_grid[my, mx])
            # nav2 uses -1 as "no information" in the costmap topic;
            # treat the same as unsafe for goal placement.
            if cost < 0:
                return False
            return cost < self._costmap_safe_max_cost
        # distance_transform mode
        return self._safe_distance[iy, ix] >= self._safety_cells


def _snap_centroid_to_safe_cell(
    raw_cx: float,
    raw_cy: float,
    cluster_xs: np.ndarray,
    cluster_ys: np.ndarray,
    safety: _SafetyChecker,
    search_radius_cells: int,
    grid_shape: Tuple[int, int],
) -> Optional[Tuple[float, float]]:
    """Find a free + safe cell near ``(raw_cx, raw_cy)``.

    Search order (matches the TB3 design intent: prefer the original
    cluster, then widen):
      1. The raw centroid cell itself if it passes :meth:`is_safe`.
      2. Cells within the cluster bbox; pick the one closest to the
         raw centroid that passes the check.
      3. A square ``search_radius_cells`` window around the raw centroid.

    Returns (cx_cell, cy_cell) or None if nothing safe is found, in
    which case the caller drops the cluster entirely.
    """
    h, w = grid_shape
    raw_ix = max(0, min(w - 1, int(round(raw_cx))))
    raw_iy = max(0, min(h - 1, int(round(raw_cy))))

    if safety.is_safe(raw_ix, raw_iy):
        return float(raw_ix), float(raw_iy)

    def _best_in_indices(
        ys: np.ndarray, xs: np.ndarray
    ) -> Optional[Tuple[float, float]]:
        if ys.size == 0:
            return None
        d2 = (xs.astype(np.float32) - raw_cx) ** 2 + (
            ys.astype(np.float32) - raw_cy
        ) ** 2
        order = np.argsort(d2)
        for idx in order:
            ix = int(xs[idx])
            iy = int(ys[idx])
            if safety.is_safe(ix, iy):
                return float(ix), float(iy)
        return None

    snap = _best_in_indices(cluster_ys, cluster_xs)
    if snap is not None:
        return snap

    x0 = max(0, raw_ix - search_radius_cells)
    x1 = min(w, raw_ix + search_radius_cells + 1)
    y0 = max(0, raw_iy - search_radius_cells)
    y1 = min(h, raw_iy + search_radius_cells + 1)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    return _best_in_indices(yy.ravel(), xx.ravel())


def compute_frontier_clusters(
    grid: np.ndarray,
    info: Any,
    robot_xy: Tuple[float, float],
    *,
    min_cluster_size: int,
    info_gain_radius_m: float,
    distance_weight: float,
    max_frontiers: int,
    safety_radius_m: float = 0.4,
    snap_search_radius_m: float = 1.0,
    costmap_grid: Optional[np.ndarray] = None,
    costmap_info: Optional[Any] = None,
    costmap_safe_max_cost: int = 75,
    debug_out: Optional[Dict[str, Any]] = None,
) -> Tuple[bool, str, List[FrontierCluster]]:
    """Detect + score + rank frontier clusters on a single OccupancyGrid.

    grid : (H, W) int array of OccupancyGrid values (0=free, -1=unknown,
           100=lethal, etc.).
    info : nav_msgs/MapMetaData-shaped object with .origin, .resolution,
           .width, .height.
    robot_xy : (rx, ry) world pose of the requester, in the same frame as
               info.origin. Used for distance scoring only.
    safety_radius_m : fallback safety distance (in metres) used when no
        ``costmap_grid`` is supplied. Centroids must be at least this far
        from the nearest occupied (>50) cell on /map.
    snap_search_radius_m : if the raw cluster centroid is unsafe, search
        this radius around it for a free + safe replacement cell before
        dropping the cluster.
    costmap_grid, costmap_info : optional /global_costmap snapshot. When
        provided, supersedes the distanceTransform fallback — a centroid
        is "safe" iff its cost in the inflated costmap is in
        ``[0, costmap_safe_max_cost)`` (TB3-style rule). The costmap may
        have a different origin / resolution / size than ``grid``; the
        per-cell safety check re-projects through world coordinates.
    costmap_safe_max_cost : upper cost threshold (exclusive). Default
        75 mirrors the TB3 paper. nav2 inflated cells use ~99 (inscribed
        inflation) and 100 (lethal); 75 leaves a margin for minor
        inflation overlap without rejecting reasonable goals.
    debug_out : optional dict to populate with 2D-debug payload. When
        not None the caller receives:
            "frontier_cells_world": List[(wx, wy)]   -- every cell tagged
                as frontier (free + ≥1 unknown 8-neighbour). Suitable
                for a CUBE_LIST overlay at scale = info.resolution.
            "rejected": List[(wx, wy, reason)]   -- centroids that did
                not make the accepted list, with a short string reason
                drawn from {"cluster_too_small", "unsafe_no_snap"}.
                Outside-bbox rejections happen in the caller and are
                appended there. The list is capped to a sane visual
                limit so RViz doesn't explode on a noisy map.
            "resolution": float   -- info.resolution mirrored for the
                caller's CUBE_LIST scale.
        Cost is ~one extra np.where + a bounded centroid-loop, never
        runs when debug_out is None — keeps the unit-test path light.

    Returns (success, message, top_clusters). top_clusters is sorted by
    score descending and trimmed to max_frontiers. Empty list is the
    legitimate "no frontier left" signal — success stays True.
    """
    # Visualisation guardrail: a noisy SLAM map can produce hundreds of
    # 1-2 cell speckle clusters. Showing every one as a red marker drowns
    # RViz; cap rejected reasons at this many entries (highest cluster_size
    # first so the operator sees the most "real-looking" rejections).
    _MAX_DEBUG_REJECTED = 50
    if info.width == 0 or info.height == 0 or info.resolution <= 0.0:
        return False, (
            f"Map has invalid info "
            f"(w={info.width} h={info.height} res={info.resolution})"
        ), []

    rx, ry = robot_xy

    free_mask = (grid == 0).astype(np.uint8)
    unknown_mask = (grid == -1).astype(np.uint8)

    # 8-neighbour 3x3 — frontier = free cell with ANY unknown in its
    # 3x3 neighbourhood (including diagonals). The TB3 paper shows this
    # catches diagonal frontiers along corners that the 4-cross misses.
    kern8 = np.ones((3, 3), dtype=np.uint8)
    unknown_neighbour = cv2.dilate(unknown_mask, kern8, iterations=1)
    frontier_mask = ((free_mask == 1) & (unknown_neighbour == 1)).astype(
        np.uint8
    )

    if frontier_mask.sum() == 0:
        if debug_out is not None:
            debug_out["frontier_cells_world"] = []
            debug_out["rejected"] = []
            debug_out["resolution"] = float(info.resolution)
        return True, "No frontier cells: environment fully explored", []

    if debug_out is not None:
        # Materialise every yellow-cell world coordinate up-front; the
        # caller renders these as a CUBE_LIST in RViz.
        ys_f, xs_f = np.where(frontier_mask == 1)
        cells_world: List[Tuple[float, float]] = []
        for cx_i, cy_i in zip(xs_f.tolist(), ys_f.tolist()):
            wx, wy = cell_to_world(float(cx_i), float(cy_i), info)
            cells_world.append((wx, wy))
        debug_out["frontier_cells_world"] = cells_world
        debug_out["rejected"] = []  # filled in inside the cluster loop
        debug_out["resolution"] = float(info.resolution)

    n_labels, labels = cv2.connectedComponents(
        frontier_mask, connectivity=8
    )

    safety = _SafetyChecker(
        grid,
        info,
        free_mask,
        safety_radius_m=safety_radius_m,
        costmap_grid=costmap_grid,
        costmap_info=costmap_info,
        costmap_safe_max_cost=costmap_safe_max_cost,
    )
    snap_search_cells = max(
        1, int(round(snap_search_radius_m / info.resolution))
    )

    radius_cells = max(1, int(round(info_gain_radius_m / info.resolution)))
    clusters: List[FrontierCluster] = []
    n_dropped_unsafe = 0
    # When debug_out is None the helper degrades to a no-op tuple list;
    # the cap is enforced post-loop so the caller can sort by size.
    debug_rejected_raw: List[Tuple[float, float, str, int]] = []
    for lbl in range(1, n_labels):
        ys, xs = np.where(labels == lbl)
        cluster_size = int(len(xs))
        raw_cx = float(np.mean(xs))
        raw_cy = float(np.mean(ys))
        if cluster_size < min_cluster_size:
            if debug_out is not None:
                # Many of these are 1-2 cell speckles — still useful to
                # see at debug time so the operator can decide whether
                # min_cluster_size is too aggressive. Cluster_size goes
                # into the 4th tuple element so we can sort + cap below.
                wx_raw, wy_raw = cell_to_world(raw_cx, raw_cy, info)
                debug_rejected_raw.append(
                    (wx_raw, wy_raw, "cluster_too_small", cluster_size)
                )
            continue

        snap = _snap_centroid_to_safe_cell(
            raw_cx, raw_cy, xs, ys, safety, snap_search_cells, grid.shape
        )
        if snap is None:
            n_dropped_unsafe += 1
            if debug_out is not None:
                wx_raw, wy_raw = cell_to_world(raw_cx, raw_cy, info)
                debug_rejected_raw.append(
                    (wx_raw, wy_raw, "unsafe_no_snap", cluster_size)
                )
            continue
        cx_cell, cy_cell = snap
        wx, wy = cell_to_world(cx_cell, cy_cell, info)

        x0 = max(0, int(round(cx_cell)) - radius_cells)
        x1 = min(info.width, int(round(cx_cell)) + radius_cells + 1)
        y0 = max(0, int(round(cy_cell)) - radius_cells)
        y1 = min(info.height, int(round(cy_cell)) + radius_cells + 1)
        info_gain = int(unknown_mask[y0:y1, x0:x1].sum())

        distance = float(math.hypot(wx - rx, wy - ry))
        score = float(info_gain) - distance_weight * distance
        clusters.append(
            (wx, wy, info_gain, distance, score, cluster_size)
        )

    if debug_out is not None:
        # Sort by cluster_size desc so the visual cap keeps the largest
        # (most interesting) rejections; drop the size before exposing
        # to the public payload — the caller doesn't need it.
        debug_rejected_raw.sort(key=lambda t: t[3], reverse=True)
        debug_out["rejected"] = [
            (wx, wy, reason)
            for (wx, wy, reason, _sz) in debug_rejected_raw[
                :_MAX_DEBUG_REJECTED
            ]
        ]

    if not clusters:
        return True, (
            f"{n_labels - 1} frontier cluster(s) found but none had a "
            f"safe centroid (safety={safety.describe()}, "
            f"snap_search={snap_search_radius_m:.2f}m, "
            f"dropped_unsafe={n_dropped_unsafe}, "
            f"min_cluster_size={min_cluster_size})."
        ), []

    clusters.sort(key=lambda c: c[4], reverse=True)
    msg = (
        f"{min(len(clusters), max_frontiers)} frontier(s) returned out "
        f"of {len(clusters)} valid clusters [safety={safety.mode}]"
    )
    if n_dropped_unsafe > 0:
        msg += f" ({n_dropped_unsafe} dropped as unsafe)"
    return True, msg + ".", clusters[:max_frontiers]


class FrontierExplorerNode(Node):
    """Cache /map; expose /get_frontiers; publish /frontier_markers."""

    def __init__(self) -> None:
        super().__init__("frontier_explorer_node")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------
        self.declare_parameter("map_topic", "/map")
        # Smallest cluster of frontier cells we are willing to return.
        # Clusters smaller than this are usually 1- or 2-cell speckles
        # at the edge of the costmap (sensor noise) and rarely worth
        # planning to.
        self.declare_parameter("min_cluster_size", 10)
        # Information-gain proxy: count unknown cells inside this
        # world-frame radius of each cluster centroid. Larger radius
        # rewards frontiers that lead into bigger unexplored pockets.
        self.declare_parameter("info_gain_radius_m", 1.5)
        # Score weight on distance. With min_cluster_size=10 a typical
        # info_gain on a fresh frontier is 30..150; weight=5.0 means
        # 1 m of extra distance "costs" 5 unknown cells of info-gain.
        self.declare_parameter("distance_weight", 5.0)
        self.declare_parameter("max_frontiers", 5)
        self.declare_parameter("marker_topic", "/frontier_markers")
        # Optional: name of the "info_gain ring" Marker namespace, kept
        # under one ns so RViz can clear it cleanly.
        self.declare_parameter("marker_ns", "frontiers")
        # Day 8 fix — minimum metric distance from a candidate centroid
        # to the nearest occupied cell on the SLAM /map. Defaults to a
        # value slightly under nav2's inflation_radius (0.5 m) so that
        # any goal we send is reachable by the global planner. Setting
        # to 0 reproduces the legacy "raw cluster mean" behaviour
        # (which can plant goals on / past walls).
        self.declare_parameter("safety_radius_m", 0.4)
        # If the raw centroid is unsafe, search this radius around it
        # for a free + safe replacement cell. 1 m is large enough to
        # snap off a thin wall (2-4 cells) without wandering into a
        # different cluster's pocket.
        self.declare_parameter("snap_search_radius_m", 1.0)
        # TB3-style: prefer the inflated nav2 costmap to validate
        # candidate centroids. When this topic is available, the
        # distanceTransform fallback is skipped — costmap cost values
        # already account for inflation, robot_radius and lethal cells.
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        # Cells with cost < this value are accepted. nav2 inscribed
        # inflation is 99, lethal is 100; 75 follows the TB3 paper and
        # leaves room for cells just inside the inflation ring without
        # rejecting otherwise reachable goals.
        self.declare_parameter("costmap_safe_max_cost", 75)
        # ---------------- Bounding-box filter (May-8 fix) -------------
        # Reject any frontier whose centroid lies outside the AABB
        # [bbox_xmin, bbox_xmax] x [bbox_ymin, bbox_ymax] in MAP frame.
        #
        # Why this exists
        #   In Isaac Sim the simulated LiDAR has ±2-3 cm per-ray noise
        #   and the warehouse walls are only 0.20 m thick. Some rays
        #   slip past the wall edge and travel out to max_laser_range
        #   (12 m), which slam_toolbox dutifully marks as `free` on
        #   the /map. Over a 5-minute mapping run the published map
        #   grew from 254x282 cells (~12 m wide, the warehouse extent)
        #   to 298x297 cells (~15 m wide) — a 5 m skirt of phantom
        #   free-space outside the real walls. The free→unknown
        #   border of that skirt then registers as a frontier; on
        #   2026-05-08 frontier_explorer returned xy=(14.99, 2.25),
        #   which is 6 m past the east wall. mapping_explorer sent
        #   that as a Nav2 goal, the planner failed (`Failed to
        #   create plan with tolerance of 0.5`), and after 4
        #   consecutive ABORTs the FSM locked into FAILED.
        #
        # Why 4 separate scalars instead of an array
        #   YAML/launch parameter loading can't infer the type of an
        #   empty array `[]` — same trap that bit collision_monitor.
        #   Sentinel values (large finite numbers acting as ±inf)
        #   sidestep the whole issue.
        #
        # Defaults
        #   ±1e9 = effectively no constraint. Override per-scene in
        #   the launch file (day8_two_phase.launch.py sets the
        #   warehouse bbox to roughly [-1, -1] .. [9, 9] in map
        #   frame).
        self.declare_parameter("bbox_xmin", -1.0e9)
        self.declare_parameter("bbox_ymin", -1.0e9)
        self.declare_parameter("bbox_xmax",  1.0e9)
        self.declare_parameter("bbox_ymax",  1.0e9)
        # ---------------- 2D frontier debug visualisation (May-8) ----
        # Frontier is fundamentally a 2D occupancy-grid concept. The
        # legacy 3D SPHERE markers float above the map and don't tell
        # the operator which cells were considered or why a candidate
        # was dropped. The new debug topics publish a flat overlay:
        #   /frontier/debug/frontier_cells       — yellow CUBE_LIST
        #   /frontier/debug/accepted_centroids   — green spheres + label
        #   /frontier/debug/rejected_centroids   — red spheres + reason
        # Default is 2D-on / 3D-off; set use_3d_frontier_debug:=true to
        # restore the legacy /frontier_markers spheres alongside.
        self.declare_parameter("use_2d_frontier_debug", True)
        self.declare_parameter("use_3d_frontier_debug", False)
        self.declare_parameter(
            "frontier_cells_topic", "/frontier/debug/frontier_cells"
        )
        self.declare_parameter(
            "accepted_centroids_topic",
            "/frontier/debug/accepted_centroids",
        )
        self.declare_parameter(
            "rejected_centroids_topic",
            "/frontier/debug/rejected_centroids",
        )
        # ---------------- Day 9+ Task 4-6 — marker hygiene ----------------
        # Without these knobs, every /get_frontiers call publishes a
        # MarkerArray that may contain hundreds of CUBEs / SPHEREs.
        # On a 5-minute mapping run RViz frame rate drops noticeably,
        # AND if frontier_explorer stops publishing (e.g. DONE) the
        # last batch lingers forever — RViz keeps painting stale yellow
        # cells / red rejected dots from the last second of mapping.
        #
        # Three orthogonal knobs:
        #   publish_frontier_debug_markers — kill switch for the entire
        #       2D debug overlay. /frontier_markers (legacy 3D) is
        #       NOT affected — that one is already gated by
        #       use_3d_frontier_debug.
        #   frontier_debug_publish_period_sec — minimum interval between
        #       debug-marker batches. Service-call cadence can be 2-3 Hz
        #       in practice (controller_server is faster than mapping
        #       needs); throttling to ~1 Hz keeps the operator view
        #       smooth without losing update freshness.
        #   max_frontier_debug_cells — cap the yellow CUBE_LIST length.
        #       Operator only needs the *general shape* of the frontier
        #       boundary, not every cell. 2000 cells at 0.05 m = a
        #       ~5 m² mosaic; plenty for human triage.
        #   publish_rejected_centroid_text — toggle the per-rejection
        #       TEXT_VIEW_FACING labels. Cheap to draw individually
        #       but adds up when 50 rejections fire every cycle.
        #   marker_lifetime_sec — explicit Marker.lifetime. RViz drops
        #       any marker older than this even if frontier_explorer
        #       crashes / stops publishing. 0 = no expiry (legacy).
        # All defaults match the values requested in the Day 9+ task
        # spec. ``frontier_explorer_status_topic`` is the input we
        # listen to to drive the marker-clear when mapping is DONE.
        self.declare_parameter(
            "publish_frontier_debug_markers", True,
        )
        self.declare_parameter(
            "frontier_debug_publish_period_sec", 1.0,
        )
        self.declare_parameter("max_frontier_debug_cells", 2000)
        self.declare_parameter("publish_rejected_centroid_text", False)
        self.declare_parameter("marker_lifetime_sec", 2.0)
        self.declare_parameter(
            "mapping_status_topic", "/mapping/status",
        )
        # When mapping_status arrives with one of these values, we
        # publish a one-shot DELETEALL on every marker topic so RViz
        # drops the lingering frontier overlay. Comma-separated string
        # so launch files can override without YAML array gymnastics.
        self.declare_parameter(
            "clear_markers_on_states", "DONE,IDLE,FAILED,CANCELLED",
        )
        # ---------------- Day 9 Task 7 — Semantic / island keep-out ----
        # Why: Go2 was driving under the table because (a) the table
        # is flagged as occupied only along the legs, leaving an
        # apparently-free pocket under the top, and (b) the frontier
        # detector happily picks that pocket because the surrounding
        # cells are unknown. Once Go2 drives in, the costmap inflates
        # the legs and we get stuck. Keep-out filters those goals
        # before they leave this node.
        # ``reject_frontiers_inside_obstacle_islands`` — drop centroids
        #     whose nearest occupied cell is within ``obstacle_island_inflation_m``,
        #     i.e. they sit inside or right next to an obstacle blob.
        # ``frontier_min_clearance_from_semantic_obstacles_m`` —
        #     reject centroids closer than this to any confirmed
        #     person/table semantic landmark.
        # ``frontier_reject_unknown_pockets`` — drop centroids that
        #     sit in unknown space surrounded by occupied cells (the
        #     "under the table" case).
        # ``obstacle_island_inflation_m`` — extra metric inflation
        #     used by the obstacle-island filter.
        self.declare_parameter(
            "reject_frontiers_inside_obstacle_islands", True,
        )
        self.declare_parameter(
            "frontier_min_clearance_from_semantic_obstacles_m", 0.8,
        )
        self.declare_parameter(
            "frontier_reject_unknown_pockets", True,
        )
        self.declare_parameter("obstacle_island_inflation_m", 0.5)
        self.declare_parameter(
            "semantic_objects_topic", "/semantic_map/objects",
        )

        self._map_topic = str(self.get_parameter("map_topic").value)
        self._min_cluster_size = int(
            self.get_parameter("min_cluster_size").value
        )
        self._info_gain_radius_m = float(
            self.get_parameter("info_gain_radius_m").value
        )
        self._distance_weight = float(
            self.get_parameter("distance_weight").value
        )
        self._max_frontiers = int(self.get_parameter("max_frontiers").value)
        self._marker_topic = str(self.get_parameter("marker_topic").value)
        self._marker_ns = str(self.get_parameter("marker_ns").value)
        self._safety_radius_m = float(
            self.get_parameter("safety_radius_m").value
        )
        self._snap_search_radius_m = float(
            self.get_parameter("snap_search_radius_m").value
        )
        self._costmap_topic = str(
            self.get_parameter("costmap_topic").value
        )
        self._costmap_safe_max_cost = int(
            self.get_parameter("costmap_safe_max_cost").value
        )
        self._bbox_xmin = float(self.get_parameter("bbox_xmin").value)
        self._bbox_ymin = float(self.get_parameter("bbox_ymin").value)
        self._bbox_xmax = float(self.get_parameter("bbox_xmax").value)
        self._bbox_ymax = float(self.get_parameter("bbox_ymax").value)
        # Treat "essentially unbounded" defaults specially so the
        # heartbeat logs say so instead of "[-1e9, 1e9]".
        self._bbox_active = (
            self._bbox_xmin > -1.0e8 or self._bbox_ymin > -1.0e8
            or self._bbox_xmax <  1.0e8 or self._bbox_ymax <  1.0e8
        )
        self._use_2d_debug = bool(
            self.get_parameter("use_2d_frontier_debug").value
        )
        self._use_3d_debug = bool(
            self.get_parameter("use_3d_frontier_debug").value
        )
        self._frontier_cells_topic = str(
            self.get_parameter("frontier_cells_topic").value
        )
        self._accepted_centroids_topic = str(
            self.get_parameter("accepted_centroids_topic").value
        )
        self._rejected_centroids_topic = str(
            self.get_parameter("rejected_centroids_topic").value
        )
        # Day 9+ Task 4-6 — marker hygiene parameters.
        self._publish_debug_markers = bool(
            self.get_parameter("publish_frontier_debug_markers").value
        )
        self._debug_publish_period_sec = float(
            self.get_parameter("frontier_debug_publish_period_sec").value
        )
        self._max_debug_cells = int(
            self.get_parameter("max_frontier_debug_cells").value
        )
        self._publish_rejected_text = bool(
            self.get_parameter("publish_rejected_centroid_text").value
        )
        self._marker_lifetime_sec = float(
            self.get_parameter("marker_lifetime_sec").value
        )
        self._mapping_status_topic = str(
            self.get_parameter("mapping_status_topic").value
        )
        clear_states_raw = str(
            self.get_parameter("clear_markers_on_states").value
        )
        self._clear_marker_states = {
            s.strip().upper()
            for s in clear_states_raw.split(",")
            if s.strip()
        }
        # When the last published debug-marker batch happened. Used to
        # enforce ``frontier_debug_publish_period_sec``. Also used by the
        # mapping-status callback to know whether a DELETEALL-clear is
        # actually needed (skip if we already published nothing).
        self._last_debug_publish_ns = 0
        # Track whether we've already cleared markers for the *current*
        # terminal mapping state. Without this we'd republish a
        # DELETEALL on every status repeat (TRANSIENT_LOCAL means we get
        # the cached value as soon as we subscribe, plus periodic
        # repeats), drowning RViz with empty marker arrays.
        self._last_mapping_state: Optional[str] = None
        # Day 9 Task 7 — semantic + island keep-out parameters.
        self._reject_inside_obstacle_islands = bool(
            self.get_parameter(
                "reject_frontiers_inside_obstacle_islands"
            ).value
        )
        self._semantic_clearance_m = float(
            self.get_parameter(
                "frontier_min_clearance_from_semantic_obstacles_m"
            ).value
        )
        self._reject_unknown_pockets = bool(
            self.get_parameter("frontier_reject_unknown_pockets").value
        )
        self._island_inflation_m = float(
            self.get_parameter("obstacle_island_inflation_m").value
        )
        self._semantic_objects_topic = str(
            self.get_parameter("semantic_objects_topic").value
        )
        # Track whether we've already cleared the legacy 3D markers
        # after starting up with use_3d_frontier_debug=False so RViz
        # doesn't keep stale spheres around forever.
        self._legacy_3d_cleared = False

        # --------------------------------------------------------------
        # State
        # --------------------------------------------------------------
        self._map: Optional[OccupancyGrid] = None
        self._costmap: Optional[OccupancyGrid] = None
        # Day 9 — confirmed semantic obstacle list, refreshed by the
        # /semantic_map/objects subscriber. We keep only the stuff we
        # use (xy + class + confirmed) so the keep-out check is a tight
        # numpy / list comprehension.
        self._semantic_obstacles: List[Tuple[float, float, str]] = []
        self._n_service_calls = 0
        self._n_returned_frontiers_total = 0

        # --------------------------------------------------------------
        # ROS infra
        # --------------------------------------------------------------
        # /map is published by slam_toolbox / map_server with
        # TRANSIENT_LOCAL durability — match it so we get the latched
        # latest map immediately on subscribe instead of waiting for
        # the next periodic re-publish.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            OccupancyGrid, self._map_topic, self._on_map, map_qos
        )
        # nav2 publishes /global_costmap/costmap with TRANSIENT_LOCAL
        # too; reuse the same QoS so we get the latched latest grid on
        # subscribe. If nav2 isn't up yet, _on_costmap simply never
        # fires and the algorithm falls back to its distanceTransform
        # safety check.
        self.create_subscription(
            OccupancyGrid, self._costmap_topic, self._on_costmap, map_qos
        )
        # Day 9 — semantic landmark keep-out feed. /semantic_map/objects
        # is published BEST_EFFORT in the aggregator (matches publishers
        # default); reuse a plain depth=10 reliable subscription so we
        # cope with either side switching to RELIABLE later.
        try:
            from go2_msgs.msg import SemanticEntityArray  # local import
            self.create_subscription(
                SemanticEntityArray,
                self._semantic_objects_topic,
                self._on_semantic_objects,
                10,
            )
            self._semantic_msg_type_ok = True
        except Exception as exc:
            self.get_logger().warn(
                f"go2_msgs/SemanticEntityArray import failed "
                f"({type(exc).__name__}: {exc}); semantic frontier "
                f"keep-out disabled. Set "
                f"frontier_min_clearance_from_semantic_obstacles_m=0 "
                f"to silence this."
            )
            self._semantic_msg_type_ok = False
        # Day 9+ Task 4 — mapping status feed. We subscribe with the
        # same TRANSIENT_LOCAL/depth=1 QoS that mapping_explorer
        # publishes with, so a late-bound subscriber (e.g. frontier
        # explorer started after mapping has already entered DONE) is
        # immediately delivered the latched terminal value and can
        # clear stale markers right away.
        mapping_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        from std_msgs.msg import String as _MapString
        self.create_subscription(
            _MapString,
            self._mapping_status_topic,
            self._on_mapping_status,
            mapping_status_qos,
        )

        self._marker_pub = self.create_publisher(
            MarkerArray, self._marker_topic, 10
        )
        # Day 8+: 2D debug overlays. Single Marker (CUBE_LIST) for the
        # yellow cells is far cheaper than a MarkerArray-of-CUBEs; we
        # use MarkerArray for centroid overlays so each sphere can ship
        # alongside its TEXT_VIEW_FACING sibling.
        self._frontier_cells_pub = self.create_publisher(
            Marker, self._frontier_cells_topic, 10
        )
        self._accepted_centroids_pub = self.create_publisher(
            MarkerArray, self._accepted_centroids_topic, 10
        )
        self._rejected_centroids_pub = self.create_publisher(
            MarkerArray, self._rejected_centroids_topic, 10
        )
        self.create_service(
            GetFrontiers, "/get_frontiers", self._on_get_frontiers
        )

        self.get_logger().info(
            f"frontier_explorer ready. map_topic={self._map_topic!r} "
            f"costmap_topic={self._costmap_topic!r} "
            f"min_cluster_size={self._min_cluster_size} "
            f"info_gain_radius={self._info_gain_radius_m:.2f}m "
            f"distance_weight={self._distance_weight:.2f} "
            f"max_frontiers={self._max_frontiers} "
            f"safety_radius={self._safety_radius_m:.2f}m "
            f"snap_search_radius={self._snap_search_radius_m:.2f}m "
            f"costmap_safe_max_cost={self._costmap_safe_max_cost} "
            + (
                f"bbox=[{self._bbox_xmin:.1f},{self._bbox_ymin:.1f},"
                f"{self._bbox_xmax:.1f},{self._bbox_ymax:.1f}]"
                if self._bbox_active else "bbox=unbounded"
            )
            + f" debug2d={self._use_2d_debug} debug3d={self._use_3d_debug} "
            + f"publish_debug_markers={self._publish_debug_markers} "
            + f"debug_period_sec={self._debug_publish_period_sec:.2f} "
            + f"max_debug_cells={self._max_debug_cells} "
            + f"reject_text={self._publish_rejected_text} "
            + f"marker_lifetime_sec={self._marker_lifetime_sec:.2f} "
            + f"clear_states={sorted(self._clear_marker_states)} "
            + f"mapping_status_topic={self._mapping_status_topic!r}"
        )

    # ------------------------------------------------------------------
    # Map / costmap callbacks
    # ------------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        if self._costmap is None:
            self.get_logger().info(
                f"global costmap latched on {self._costmap_topic!r} "
                f"({msg.info.width}x{msg.info.height} "
                f"res={msg.info.resolution:.3f}m); centroid safety "
                f"check switched to costmap mode."
            )
        self._costmap = msg

    # ------------------------------------------------------------------
    # Day 9+ Task 4 — clear frontier markers when mapping ends
    # ------------------------------------------------------------------
    def _on_mapping_status(self, msg) -> None:
        """Drop stale frontier overlays when mapping enters a terminal
        state.

        ``mapping_explorer`` publishes "IDLE" / "NAVIGATING" / "DONE" /
        "FAILED:<reason>". We only care about the leading verb (split
        on ':' to drop the failure detail) so a status like
        "FAILED:no_TF" still matches the "FAILED" clear-set.
        """
        raw = (msg.data or "").strip().upper()
        if not raw:
            return
        verb = raw.split(":", 1)[0].strip()
        if verb == self._last_mapping_state:
            # Same state as last time — TRANSIENT_LOCAL keeps replaying
            # the cached value, but the marker clear is a one-shot.
            return
        self._last_mapping_state = verb
        if verb in self._clear_marker_states:
            # Re-arm the throttle clock so a /get_frontiers call that
            # races with the status transition still publishes once.
            self._last_debug_publish_ns = 0
            self.get_logger().info(
                f"frontier_explorer: mapping_status={verb!r} -> "
                f"clearing all frontier markers (DELETEALL)"
            )
            self._clear_all_frontier_markers()

    def _clear_all_frontier_markers(self) -> None:
        """Push DELETEALL on every marker topic this node owns.

        Touches /frontier_markers (legacy 3D) and the three 2D debug
        topics. Cheap (one message per topic). RViz DELETEALL clears
        every namespace bound to that publisher, so we don't need to
        enumerate ns / id pairs.
        """
        # Use a stable frame_id for the clear marker. We do NOT need
        # the actual map frame here — DELETEALL ignores frame, but
        # RViz still warns if frame_id is empty, so use the cached
        # /map frame (or "map" as a sane fallback if /map hasn't
        # been seen yet).
        frame_id = (
            self._map.header.frame_id
            if self._map is not None and self._map.header.frame_id
            else "map"
        )
        now = self.get_clock().now().to_msg()

        def _delete_array_pub(pub) -> None:
            ma = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            clear.header.frame_id = frame_id
            clear.header.stamp = now
            ma.markers.append(clear)
            pub.publish(ma)

        def _delete_marker_pub(pub) -> None:
            m = Marker()
            m.action = Marker.DELETEALL
            m.header.frame_id = frame_id
            m.header.stamp = now
            pub.publish(m)

        _delete_array_pub(self._marker_pub)
        _delete_marker_pub(self._frontier_cells_pub)
        _delete_array_pub(self._accepted_centroids_pub)
        _delete_array_pub(self._rejected_centroids_pub)

    def _on_semantic_objects(self, msg) -> None:
        """Refresh the confirmed-landmark cache used by the keep-out
        filter (Day 9 Task 7).

        We only count entities whose ``display_name`` final field is
        a real anchor (``pc_*`` or ``isl_*``). Confirmed-but-anchorless
        and invalid entities are excluded so a phantom person doesn't
        block a real frontier goal.
        """
        new: List[Tuple[float, float, str]] = []
        for ent in msg.entities:
            dn = (ent.display_name or "")
            parts = dn.split("|")
            if len(parts) < 3:
                continue
            status = parts[1].lower()
            anchor = parts[2].strip()
            if status != "confirmed":
                continue
            if not anchor or anchor == "-":
                continue
            new.append(
                (
                    float(ent.pose_map.position.x),
                    float(ent.pose_map.position.y),
                    str(ent.class_label),
                )
            )
        self._semantic_obstacles = new

    # ------------------------------------------------------------------
    # Day 9 Task 7 — semantic / island keep-out filter
    # ------------------------------------------------------------------
    def _apply_keep_out_filter(
        self,
        clusters: List[FrontierCluster],
        grid: np.ndarray,
        info: Any,
        debug_out: Optional[Dict[str, Any]],
    ) -> Tuple[List[FrontierCluster], int, int, int]:
        """Filter out frontier centroids that should not be navigation
        goals because they sit (a) inside an obstacle island, (b) too
        close to a confirmed person/table semantic landmark, or (c) in
        an "unknown pocket" surrounded by occupied cells (e.g. under a
        table).

        Returns ``(kept, n_dropped_island, n_dropped_semantic,
        n_dropped_unknown_pocket)``.
        """
        if not self._reject_inside_obstacle_islands and (
            self._semantic_clearance_m <= 0.0
        ) and not self._reject_unknown_pockets:
            return clusters, 0, 0, 0

        kept: List[FrontierCluster] = []
        n_drop_island = 0
        n_drop_semantic = 0
        n_drop_unknown = 0

        # Pre-compute distance-to-nearest-occupied (in metres) for the
        # island filter. Only need this if the inflation knob is set.
        dist_m = None
        if (
            self._reject_inside_obstacle_islands
            and self._island_inflation_m > 0.0
        ):
            occ = (grid >= 100).astype(np.uint8)
            if occ.sum() > 0:
                non_obstacle = (1 - occ).astype(np.uint8)
                # cv2 distanceTransform reports cells; convert to metres.
                dist_cells = cv2.distanceTransform(
                    non_obstacle, distanceType=cv2.DIST_L2, maskSize=3,
                )
                dist_m = dist_cells * float(info.resolution)

        # Pre-compute "unknown pocket" mask for centroid hit-test.
        # Definition: cell is unknown (-1) AND ≥ ``pocket_thresh``
        # of its 5×5 neighbours are occupied (>= 100). 0.4 (40%) is
        # the empirical threshold below which the open-warehouse
        # frontier doesn't trip; tables/walls cross 0.4 easily.
        pocket_mask = None
        if self._reject_unknown_pockets:
            unk = (grid == -1).astype(np.uint8)
            occ = (grid >= 100).astype(np.uint8)
            if unk.sum() > 0:
                kern = np.ones((5, 5), dtype=np.float32)
                occ_neighbour = cv2.filter2D(
                    occ.astype(np.float32),
                    ddepth=-1,
                    kernel=kern,
                    borderType=cv2.BORDER_CONSTANT,
                )
                pocket_mask = (unk == 1) & (occ_neighbour >= 5.0)

        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        res = float(info.resolution)

        for cl in clusters:
            wx, wy = float(cl[0]), float(cl[1])
            cell_x = int((wx - ox) / res)
            cell_y = int((wy - oy) / res)
            in_grid = (
                0 <= cell_x < info.width and 0 <= cell_y < info.height
            )
            # (a) Inside an obstacle island.
            if (
                self._reject_inside_obstacle_islands
                and dist_m is not None and in_grid
            ):
                if dist_m[cell_y, cell_x] < self._island_inflation_m:
                    n_drop_island += 1
                    if debug_out is not None:
                        debug_out["rejected"].append(
                            (wx, wy, "inside_obstacle_island")
                        )
                    continue
            # (c) Unknown pocket surrounded by occupied — same code path
            #     but the test uses the pre-computed mask.
            if (
                self._reject_unknown_pockets
                and pocket_mask is not None and in_grid
            ):
                if bool(pocket_mask[cell_y, cell_x]):
                    n_drop_unknown += 1
                    if debug_out is not None:
                        debug_out["rejected"].append(
                            (wx, wy, "unknown_pocket")
                        )
                    continue
            # (b) Within clearance of a confirmed person/table landmark.
            if self._semantic_clearance_m > 0.0 and self._semantic_obstacles:
                clr2 = self._semantic_clearance_m * self._semantic_clearance_m
                hit = False
                for sx, sy, _cls in self._semantic_obstacles:
                    if (sx - wx) ** 2 + (sy - wy) ** 2 < clr2:
                        hit = True
                        break
                if hit:
                    n_drop_semantic += 1
                    if debug_out is not None:
                        debug_out["rejected"].append(
                            (
                                wx, wy,
                                "near_semantic_obstacle",
                            )
                        )
                    continue
            kept.append(cl)
        return kept, n_drop_island, n_drop_semantic, n_drop_unknown

    # ------------------------------------------------------------------
    # Service implementation
    # ------------------------------------------------------------------
    def _on_get_frontiers(
        self,
        request: GetFrontiers.Request,
        response: GetFrontiers.Response,
    ) -> GetFrontiers.Response:
        self._n_service_calls += 1

        if self._map is None:
            response.success = False
            response.message = (
                f"No map received yet on {self._map_topic!r}"
            )
            self.get_logger().warn(response.message)
            return response

        info = self._map.info
        # Frame mismatch is a soft warn — caller is responsible for
        # posting the request in the map's frame.
        map_frame = self._map.header.frame_id or "map"
        req_frame = request.robot_pose.header.frame_id or map_frame
        if req_frame != map_frame:
            self.get_logger().warn(
                f"robot_pose frame_id={req_frame!r} != map frame "
                f"{map_frame!r}; assuming caller already transformed."
            )

        rx = float(request.robot_pose.pose.position.x)
        ry = float(request.robot_pose.pose.position.y)
        grid = np.array(self._map.data, dtype=np.int16).reshape(
            info.height, info.width
        )

        costmap_grid: Optional[np.ndarray] = None
        costmap_info = None
        if self._costmap is not None:
            costmap_info = self._costmap.info
            costmap_grid = np.array(
                self._costmap.data, dtype=np.int16
            ).reshape(costmap_info.height, costmap_info.width)

        # debug_out is opt-in — populated only when the 2D overlay is
        # enabled, so the unit-test code path (`debug_out=None`) keeps
        # its zero-overhead behaviour.
        debug_out: Optional[Dict[str, Any]] = (
            {} if self._use_2d_debug else None
        )

        success, message, top = compute_frontier_clusters(
            grid,
            info,
            (rx, ry),
            min_cluster_size=self._min_cluster_size,
            info_gain_radius_m=self._info_gain_radius_m,
            distance_weight=self._distance_weight,
            max_frontiers=self._max_frontiers,
            safety_radius_m=self._safety_radius_m,
            snap_search_radius_m=self._snap_search_radius_m,
            costmap_grid=costmap_grid,
            costmap_info=costmap_info,
            costmap_safe_max_cost=self._costmap_safe_max_cost,
            debug_out=debug_out,
        )

        response.success = success
        response.message = message

        if not success:
            self.get_logger().warn(message)
            # Even on failure we want the operator to see what cells
            # WERE classified as frontier (often the failure says
            # "0 safe centroids" — the cells field still informs the
            # next safety_radius / snap_search tweak).
            if debug_out is not None:
                self._publish_2d_debug(map_frame, debug_out, [])
            return response

        # Bounding-box filter — May-8 fix for the "frontier in the
        # phantom skirt outside the warehouse walls" failure mode.
        # Drop any centroid that escapes the operator-defined AABB.
        # Done AFTER compute_frontier_clusters so it doesn't affect
        # cluster discovery (operator may want to widen bbox at
        # runtime via `ros2 param set` and immediately see frontiers
        # surface) and BEFORE truncation to max_frontiers so we
        # never publish a high-scoring out-of-bounds frontier.
        if self._bbox_active:
            n_before = len(top)
            kept: List[FrontierCluster] = []
            for t in top:
                in_bbox = (
                    self._bbox_xmin <= t[0] <= self._bbox_xmax
                    and self._bbox_ymin <= t[1] <= self._bbox_ymax
                )
                if in_bbox:
                    kept.append(t)
                elif debug_out is not None:
                    debug_out["rejected"].append(
                        (float(t[0]), float(t[1]), "outside_bbox")
                    )
            top = kept
            n_after = len(top)
            if n_after < n_before:
                # Append rejection note to the operator message so
                # mapping_explorer's heartbeat surfaces this clearly
                # ("X frontiers ... out of Y, dropped Z bbox").
                response.message = (
                    f"{message[:-1] if message.endswith('.') else message}"
                    f", dropped {n_before - n_after} centroid(s) "
                    f"outside bbox=[{self._bbox_xmin:.1f},"
                    f"{self._bbox_ymin:.1f},{self._bbox_xmax:.1f},"
                    f"{self._bbox_ymax:.1f}]."
                )
                message = response.message  # for the log line below

        # Day 9 Task 7 — semantic + obstacle island keep-out. Run after
        # bbox filter so an out-of-bbox cluster is reported as such
        # rather than as semantic-near-obstacle.
        n_pre_keepout = len(top)
        top, n_drop_isl, n_drop_sem, n_drop_unk = (
            self._apply_keep_out_filter(top, grid, info, debug_out)
        )
        if (n_drop_isl + n_drop_sem + n_drop_unk) > 0:
            response.message = (
                f"{message[:-1] if message.endswith('.') else message}"
                f", dropped {n_drop_isl} obstacle-island, "
                f"{n_drop_sem} near-semantic, "
                f"{n_drop_unk} unknown-pocket centroid(s)."
            )
            message = response.message
            self.get_logger().debug(
                f"keep-out: pre={n_pre_keepout} post={len(top)} "
                f"island={n_drop_isl} semantic={n_drop_sem} "
                f"unknown_pocket={n_drop_unk} "
                f"clearance={self._semantic_clearance_m:.2f}m "
                f"island_inflation={self._island_inflation_m:.2f}m "
                f"semantic_obstacles={len(self._semantic_obstacles)}"
            )

        if not top:
            self._publish_markers(map_frame, [], [])
            if debug_out is not None:
                self._publish_2d_debug(map_frame, debug_out, [])
            self.get_logger().info(
                f"GetFrontiers: 0 frontiers ({message}). "
                f"calls_total={self._n_service_calls}"
            )
            return response

        goals: List[PoseStamped] = []
        scores: List[float] = []
        info_gains: List[int] = []
        distances: List[float] = []
        for wx, wy, ig, d, s, _sz in top:
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = map_frame
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            goals.append(ps)
            scores.append(float(s))
            info_gains.append(int(ig))
            distances.append(float(d))

        response.frontier_goals = goals
        response.scores = scores
        response.info_gains = info_gains
        response.distances = distances

        self._n_returned_frontiers_total += len(goals)
        self._publish_markers(map_frame, goals, scores)
        if debug_out is not None:
            # Pass the accepted top through so the 2D layer can render
            # green spheres co-located with the legacy 3D ones.
            self._publish_2d_debug(map_frame, debug_out, top)

        self.get_logger().info(
            f"GetFrontiers: returned {len(goals)} (best "
            f"score={scores[0]:.1f}, info_gain={info_gains[0]}, "
            f"dist={distances[0]:.2f}m). calls_total="
            f"{self._n_service_calls} returned_total="
            f"{self._n_returned_frontiers_total}"
        )
        return response

    def _publish_markers(
        self,
        frame_id: str,
        goals: List[PoseStamped],
        scores: List[float],
    ) -> None:
        # Legacy 3D markers. With the new 2D debug overlay, these are
        # opt-in (use_3d_frontier_debug:=true). When disabled we publish
        # exactly one DELETEALL on startup so RViz drops any spheres
        # left over from a previous run with the legacy default.
        if not self._use_3d_debug:
            if not self._legacy_3d_cleared:
                ma = MarkerArray()
                clear = Marker()
                clear.action = Marker.DELETEALL
                clear.header.frame_id = frame_id
                clear.header.stamp = self.get_clock().now().to_msg()
                ma.markers.append(clear)
                self._marker_pub.publish(ma)
                self._legacy_3d_cleared = True
            return

        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.frame_id = frame_id
        clear.header.stamp = self.get_clock().now().to_msg()
        ma.markers.append(clear)

        # Day 9+ Task 5 — finite lifetime so RViz drops the legacy 3D
        # spheres if frontier_explorer stops publishing.
        from builtin_interfaces.msg import Duration as _DurMsg
        lifetime_sec = max(0.0, self._marker_lifetime_sec)
        if lifetime_sec > 0.0:
            secs = int(lifetime_sec)
            nsecs = int((lifetime_sec - secs) * 1e9)
            life = _DurMsg(sec=secs, nanosec=nsecs)
        else:
            life = _DurMsg(sec=0, nanosec=0)

        if goals:
            s_lo = min(scores)
            s_hi = max(scores)
            s_span = max(1e-6, s_hi - s_lo)
            for i, (ps, s) in enumerate(zip(goals, scores)):
                t = (s - s_lo) / s_span  # 0 = worst, 1 = best
                # Best = green, worst = red, linear in between.
                color = ColorRGBA(
                    r=float(1.0 - t),
                    g=float(t),
                    b=0.1,
                    a=0.9,
                )
                m = Marker()
                m.header.frame_id = frame_id
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = self._marker_ns
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose = ps.pose
                m.pose.position.z = 0.15
                m.scale.x = m.scale.y = m.scale.z = 0.30
                m.color = color
                m.lifetime = life
                ma.markers.append(m)

                txt = Marker()
                txt.header = m.header
                txt.ns = self._marker_ns + "_label"
                txt.id = i
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                txt.pose = ps.pose
                txt.pose.position.z = 0.55
                txt.scale.z = 0.18
                txt.color.r = 1.0
                txt.color.g = 1.0
                txt.color.b = 1.0
                txt.color.a = 0.95
                txt.text = f"#{i} s={s:.0f}"
                txt.lifetime = life
                ma.markers.append(txt)

        self._marker_pub.publish(ma)

    # ------------------------------------------------------------------
    # 2D debug overlay (May-8) — yellow cells + green/red centroids
    # ------------------------------------------------------------------
    def _publish_2d_debug(
        self,
        frame_id: str,
        debug: Dict[str, Any],
        accepted_top: List[FrontierCluster],
    ) -> None:
        """Publish the three /frontier/debug/* topics from a debug dict.

        ``debug`` is the dict populated by compute_frontier_clusters
        (keys: frontier_cells_world, rejected, resolution).
        ``accepted_top`` is the post-bbox accepted cluster list — used
        for the green centroids + score labels.

        Day 9+ Task 4-6 — three guardrails applied here:
        1. ``publish_frontier_debug_markers=False`` → no-op.
        2. ``frontier_debug_publish_period_sec`` throttle: skip if the
           previous batch is more recent than the configured period.
           Mapping_status DONE/IDLE handler resets the clock to 0 to
           guarantee the final "no markers" frame goes out.
        3. ``max_frontier_debug_cells`` truncates the yellow CUBE_LIST
           and ``publish_rejected_centroid_text`` toggles per-rejection
           labels, both to keep RViz fluid on noisy maps.
        """
        if not self._publish_debug_markers:
            return
        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(self._debug_publish_period_sec * 1e9)
        if (
            period_ns > 0
            and self._last_debug_publish_ns > 0
            and (now_ns - self._last_debug_publish_ns) < period_ns
        ):
            return
        self._last_debug_publish_ns = now_ns

        now = self.get_clock().now().to_msg()
        resolution = float(debug.get("resolution", 0.05))
        # Convert the lifetime knob into a Duration once so each marker
        # below can copy it without recomputing. 0 means "no expiry".
        lifetime_sec = max(0.0, self._marker_lifetime_sec)
        from builtin_interfaces.msg import Duration as _DurMsg
        if lifetime_sec > 0.0:
            secs = int(lifetime_sec)
            nsecs = int((lifetime_sec - secs) * 1e9)
            life = _DurMsg(sec=secs, nanosec=nsecs)
        else:
            life = _DurMsg(sec=0, nanosec=0)

        # --- A. Yellow CUBE_LIST of frontier cells ---------------------
        cells = debug.get("frontier_cells_world", [])
        cube_marker = Marker()
        cube_marker.header.frame_id = frame_id
        cube_marker.header.stamp = now
        cube_marker.ns = "frontier_cells"
        cube_marker.id = 0
        cube_marker.type = Marker.CUBE_LIST
        cube_marker.action = Marker.ADD
        # CUBE_LIST stores per-cube position in `points`; scale is the
        # uniform per-cube size. Match map resolution so cells tile flat
        # across the occupancy grid without overlap.
        cube_marker.scale.x = resolution
        cube_marker.scale.y = resolution
        cube_marker.scale.z = 0.02
        cube_marker.color = ColorRGBA(r=1.0, g=0.92, b=0.16, a=0.6)
        cube_marker.pose.orientation.w = 1.0
        cube_marker.lifetime = life
        # Cap to ``max_frontier_debug_cells`` to keep RViz responsive
        # on huge maps. Operator only needs the rough shape of the
        # frontier boundary; sampling stride preserves the spatial
        # distribution rather than just clipping the head of the list.
        if (
            self._max_debug_cells > 0
            and len(cells) > self._max_debug_cells
        ):
            stride = max(1, len(cells) // self._max_debug_cells)
            cells_iter = cells[::stride]
        else:
            cells_iter = cells
        for wx, wy in cells_iter:
            cube_marker.points.append(
                Point(x=float(wx), y=float(wy), z=0.03)
            )
        # Empty CUBE_LIST silently no-ops in RViz; no need to special-case.
        self._frontier_cells_pub.publish(cube_marker)

        # --- B. Accepted centroids (green) -----------------------------
        ma_acc = MarkerArray()
        clear_acc = Marker()
        clear_acc.action = Marker.DELETEALL
        clear_acc.header.frame_id = frame_id
        clear_acc.header.stamp = now
        ma_acc.markers.append(clear_acc)
        for i, fc in enumerate(accepted_top):
            wx, wy, info_gain, _dist, score, cluster_size = fc
            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = now
            sphere.ns = "frontier_accepted"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(wx)
            sphere.pose.position.y = float(wy)
            sphere.pose.position.z = 0.08
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.18
            sphere.color = ColorRGBA(r=0.10, g=0.85, b=0.20, a=0.95)
            sphere.lifetime = life
            ma_acc.markers.append(sphere)

            label = Marker()
            label.header = sphere.header
            label.ns = "frontier_accepted_label"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(wx)
            label.pose.position.y = float(wy)
            label.pose.position.z = 0.30
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            label.text = (
                f"#{i} s={score:.0f} ig={info_gain} sz={cluster_size}"
            )
            label.lifetime = life
            ma_acc.markers.append(label)
        self._accepted_centroids_pub.publish(ma_acc)

        # --- C. Rejected centroids (red) -------------------------------
        ma_rej = MarkerArray()
        clear_rej = Marker()
        clear_rej.action = Marker.DELETEALL
        clear_rej.header.frame_id = frame_id
        clear_rej.header.stamp = now
        ma_rej.markers.append(clear_rej)
        for i, item in enumerate(debug.get("rejected", [])):
            wx, wy, reason = item
            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = now
            sphere.ns = "frontier_rejected"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(wx)
            sphere.pose.position.y = float(wy)
            sphere.pose.position.z = 0.08
            sphere.pose.orientation.w = 1.0
            # Slightly smaller than accepted so they fade into the
            # background unless the operator looks for them.
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.14
            sphere.color = ColorRGBA(r=0.85, g=0.10, b=0.10, a=0.85)
            sphere.lifetime = life
            ma_rej.markers.append(sphere)

            # Per-rejection labels are spammy on cluttered maps. Gate
            # behind ``publish_rejected_centroid_text`` (default off).
            if not self._publish_rejected_text:
                continue
            label = Marker()
            label.header = sphere.header
            label.ns = "frontier_rejected_label"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(wx)
            label.pose.position.y = float(wy)
            label.pose.position.z = 0.26
            label.pose.orientation.w = 1.0
            label.scale.z = 0.13
            label.color = ColorRGBA(r=1.0, g=0.6, b=0.6, a=0.9)
            label.text = str(reason)
            label.lifetime = life
            ma_rej.markers.append(label)
        self._rejected_centroids_pub.publish(ma_rej)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
