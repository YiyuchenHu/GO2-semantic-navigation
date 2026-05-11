"""Day 6 — semantic_memory_aggregator_node.

Subscribes
----------
/detections_3d (vision_msgs/Detection3DArray)
    Per-frame 3D detections from depth_projector_node, in the
    target frame (default `map`).

Publishes
---------
/semantic_map/objects (go2_msgs/SemanticEntityArray)
    Persistent object registry. Each entity carries a stable
    string id, class label, current position estimate, observation
    count, first/last seen timestamps, and a confidence in [0, 1]
    that decays toward zero when the object stops being observed.
    Re-uses the project's existing Phase-2 message schema so Day 7+
    target selection / goal generation can drop in the new
    aggregator with minimal code change.

/semantic_map/markers (visualization_msgs/MarkerArray)
    RViz: all **confirmed** landmarks (visible + remembered combined).
    Legacy topic kept for backward compatibility.

/semantic_map/markers_visible (optional MarkerArray)
    Confirmed landmarks with ``currently_visible=True`` when
    ``publish_split_visibility_markers`` is True.

/semantic_map/markers_remembered (optional MarkerArray)
    Confirmed landmarks with ``currently_visible=False`` under the
    same gate.

/semantic_map/debug_markers (MarkerArray)
    Candidates, invalid, and confirmed-without-anchor rejects.
-----------------------------------------------------
Different update cadences:
  * depth_projector ticks at every synchronised RGB-D-detection
    triple (~14 Hz).
  * semantic_memory ticks once per /detections_3d message AND a
    slower housekeeping timer (1 Hz) for confidence decay and
    expired-entity pruning. Stateful logic.
Different failure modes:
  * depth_projector fails on TF / depth NaN — geometry bugs.
  * semantic_memory fails on data-association — algorithmic bugs.
  Mixing them makes both painful to bisect.

Why this node is *also* separate from the legacy Phase-2
``semantic_map_node``: that one's input is the chair-only
``go2_msgs/ObjectObservationArray`` from
``object_localizer_3d_node``. Day 5 + 6 standardise on
``vision_msgs/Detection3DArray`` so multiple classes flow through
without aliasing logic. Phase 2 stays in the tree for the legacy
chair-only launch tree but is **deprecated** (see
``docs/known_issues.md`` ADR-005).

Aggregation algorithm
---------------------
On each /detections_3d message, for each Detection3D:
  1. Shortlist existing entities with the same class label.
  2. Find the closest one within `nms_radius_m` (default 0.3 m).
  3. **If found** — merge:
       * Update its position with an exponential moving average
         (configurable `position_alpha`); a fresh observation
         pushes the estimate ~30% of the way toward the new sample.
       * Bump observation_count.
       * Increase confidence by `confidence_step_up` (capped at 1.0).
       * Refresh last_seen and currently_visible.
  4. **Otherwise** — register a new entity:
       * Mint a UUID4 string id.
       * Inherit class from the detection.
       * Confidence starts at the detection's score, capped at 1.0.

A 1 Hz housekeeping timer:
  * Multiplies every entity's confidence by `confidence_decay_per_sec`
    (default 0.95 ⇒ ~50% decay over 13 s).
  * Marks `currently_visible = False` when an entity hasn't been
    matched in `visibility_timeout_sec` seconds.
  * Drops entities whose confidence falls below
    `prune_confidence_threshold` (default 0.05) AND whose
    last_seen is older than `prune_age_sec` (default 30 s).
  * Promotes any entity with `observations_count >=
    permanent_after_observations` to a permanent landmark: from
    that point on, decay and pruning are skipped, so its position
    survives long Go2 traverses, occlusions, and the rest of the
    session. Set the parameter to 0 to disable.
  * Performs a same-class second-pass merge: any two same-class
    entities within `entity_merge_radius_m` are fused into one
    (observation-count-weighted position, max confidence, sum of
    counts). This catches the "one desk → desk_001 + desk_002"
    failure that happens when projection jitter makes two
    consecutive detections fall on opposite sides of `nms_radius_m`.
    Set the parameter to 0 to disable.

The MVP intentionally avoids:
  * Kalman filter on entity position (overkill for a static object
    in a 10 m room; revisit when dynamic objects matter).
  * Per-entity 3D bounding boxes (BoundingBox3D from upstream is
    already zero-extent; we treat entities as point landmarks).
  * Cross-class matching (no chair → couch alias logic; that's
    YOLOE's job via prompts).

Outputs are published on every input message AND on every
housekeeping tick, so a downstream consumer that polls only
/semantic_map/objects gets at least 1 Hz refresh even with no
detections.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from go2_msgs.msg import SemanticEntity, SemanticEntityArray
from geometry_msgs.msg import Point, PointStamped, Vector3
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_point
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from .semantic_memory_policy_helpers import (
    candidate_not_confirmed_hint_from_rejection,
    confirmed_split_visibility_bucket,
    effective_detection_confidence_floor,
    merge_quality_tuple,
    promotion_blocked_without_anchor,
    table_candidate_not_confirmed_tag,
    table_promote_via_pc_anchor_path,
)


# ---------------------------------------------------------------------------
# Canonical class map (Task 4)
# ---------------------------------------------------------------------------
# YOLOE will sometimes label a workbench/desk as "desk" and sometimes as
# "table" depending on viewpoint and prompt. For navigation, the user
# command "go to table" should match either.  We canonicalise the class
# at *aggregator* level so:
#   - one entity per physical object (desk + table observations merge)
#   - target_selector can keep a simple equality match on class_label
#   - the original detector label stays available as `display_name` for
#     RViz / debug.
#
# Keys are normalised (lower-case, spaces → underscores). Values are the
# canonical class name. Add aliases as needed.
_DEFAULT_CANONICAL_CLASS_MAP: Dict[str, str] = {
    # person family
    "person": "person",
    "human": "person",
    "man": "person",
    "woman": "person",
    "people": "person",
    "pedestrian": "person",
    "worker": "person",
    "construction_worker": "person",
    # table family — desk / dining table / workbench all collapse to table
    "table": "table",
    "desk": "table",
    "dining_table": "table",
    "workbench": "table",
    "office_desk": "table",
    # chair family — keep distinct, just normalise office_chair
    "chair": "chair",
    "office_chair": "chair",
}


def _normalise_class_key(s: str) -> str:
    return (s or "").strip().lower().replace(" ", "_")




# ---------------------------------------------------------------------------
# PointCloud cluster association (Day 9 Task 1)
# ---------------------------------------------------------------------------
# Why PointCloud2 anchors at all? Day 8 relied solely on /map obstacle
# islands, but tables and people are routinely under-represented in the
# 2D occupancy grid: thin table legs reflect LiDAR poorly, and a person
# standing still produces only a sliver of occupied cells in slam_toolbox
# until the robot translates around them. Yet in /lidar/points the same
# objects produce dense, distinct point clusters from the very first
# frame. The cluster associator is the primary anchor (faster + denser);
# /map island anchoring drops to fallback / cross-validation.
#
# Algorithm:
#   1. Maintain a small ring buffer of {stamp, source_frame, points_xyz}
#      transformed into the map frame. We transform once per cloud, not
#      once per detection, since detection counts >> cloud counts.
#   2. For each detection, slice points within (search_radius, z-range).
#   3. Voxel-grid greedy clustering: discretise XY at ``cluster_tolerance_m``
#      resolution, BFS connected components, pick the cluster with the
#      smallest (centroid_xy − detection_xy) distance.
#   4. Return centroid + cluster_id (a stable hash of the centroid grid)
#      so duplicate detections that hit the same physical cluster snap
#      to the same anchor.
@dataclass
class PointCloudResult:
    """Outcome of trying to anchor a 3D detection on a LiDAR cluster.

    success:
        True iff a valid cluster was found near the detection.
    snapped_x / snapped_y / snapped_z:
        World-frame XYZ of the cluster centroid; the Z is the *median*
        of the cluster's points so a person snaps near torso height
        rather than to whichever extreme their feet/head produce.
    cluster_id:
        Stable hash of the discretised centroid: ``"pc_{ix:+05d}_{iy:+05d}"``.
        Two detections projecting close enough to the same cluster will
        share this id and be merged by the existing same-island merge
        path (which we extend in this patch to fire on pc_ ids too).
    point_count:
        Number of points that ended up inside the cluster.
    bbox_length_m / bbox_width_m / bbox_height_m:
        Axis-aligned bounding-box extents of the cluster, used by the
        wall-shape filter below and surfaced to the operator for
        debugging.
    rejection_reason:
        Empty when ``success=True``; otherwise one of:
          ``no_pointcloud``                — buffer empty
          ``pointcloud_tf_failed``         — couldn't transform to map
          ``no_points_near_detection``     — radius hit zero in-range pts
          ``cluster_too_small``            — < min_cluster_points
          ``cluster_too_large``            — > max_cluster_points
          ``cluster_height_invalid``       — class-specific z range miss
          ``cluster_shape_invalid``        — wall-aspect-ratio exceeded
    """

    success: bool
    snapped_x: float
    snapped_y: float
    snapped_z: float
    cluster_id: str
    point_count: int
    bbox_length_m: float
    bbox_width_m: float
    bbox_height_m: float
    rejection_reason: str


class PointCloudClusterAssociator:
    """Snap an observation to the nearest LiDAR cluster.

    Design choices:
      * No PCL dependency, no scipy KDTree. The ring buffer stores
        decimated map-frame point arrays; a class-radius slice on a
        ~3-5 k point cloud is a fast numpy bool mask. Clustering uses
        a 2D voxel grid + iterative BFS — empirically <2 ms on the
        warehouse demo.
      * Cluster id is *position-stable* (centroid quantised to
        ``cluster_tolerance_m``). Two consecutive detections projecting
        to the same physical cluster therefore yield the same id; the
        existing same-island merge path picks them up automatically
        once we accept ``pc_*`` ids alongside ``isl_*``.
      * Per-class kwargs (``search_radius``, ``z_min``, ``z_max``,
        ``min_cluster_points``, ``max_cluster_points``) keep the
        defaults targeted at person/table while still allowing other
        classes to flow through with global defaults.
    """

    @dataclass
    class _Frame:
        """One transformed point cloud in the map frame."""
        stamp_ns: int
        # Nx3 float32 array of XYZ in map frame. We keep a numpy array
        # here so the per-detection slice is a contiguous mask op.
        points: np.ndarray

    def __init__(
        self,
        target_frame: str = "map",
        max_buffer_size: int = 4,
        max_points_per_cloud: int = 20000,
    ) -> None:
        self._target_frame = target_frame
        self._max_buffer = max(1, int(max_buffer_size))
        self._max_pts_per_cloud = max(1024, int(max_points_per_cloud))
        self._frames: Deque["PointCloudClusterAssociator._Frame"] = deque(
            maxlen=self._max_buffer
        )

    def has_pointcloud(self) -> bool:
        return len(self._frames) > 0

    def update(
        self,
        msg: PointCloud2,
        tf_buffer: Buffer,
        logger=None,
    ) -> bool:
        """Read points from ``msg`` and transform into the map frame.

        Returns True on success. On TF failure the frame is dropped
        (logged at WARN, throttled by the caller).
        """
        try:
            # Single TF lookup per cloud, not per point. We compose the
            # whole cloud as a homogeneous transform in numpy below.
            t = tf_buffer.lookup_transform(
                self._target_frame,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            if logger is not None:
                logger.warn(
                    f"PointCloudClusterAssociator: TF "
                    f"{msg.header.frame_id} -> {self._target_frame} "
                    f"failed ({type(exc).__name__}: {exc}); cloud "
                    f"dropped.",
                    throttle_duration_sec=2.0,
                )
            return False

        # Build a 4x4 homogeneous transform from the ROS Transform.
        tx, ty, tz = (
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        )
        qx, qy, qz, qw = (
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        )
        # Quaternion -> 3x3 rotation matrix.
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        R = np.array(
            [
                [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
                [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
                [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
            ],
            dtype=np.float32,
        )
        translation = np.array([tx, ty, tz], dtype=np.float32)

        # Decimate while reading to bound memory/CPU. read_points is a
        # generator over numpy structured records; itertools.islice
        # avoids materialising the entire cloud when not needed.
        try:
            raw = point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True,
            )
        except Exception:
            # Older sensor_msgs_py versions don't have read_points_numpy.
            arr = np.fromiter(
                (
                    (p[0], p[1], p[2])
                    for p in point_cloud2.read_points(
                        msg,
                        field_names=("x", "y", "z"),
                        skip_nans=True,
                    )
                ),
                dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")]),
                count=-1,
            )
            raw = np.stack(
                (arr["x"], arr["y"], arr["z"]), axis=-1
            ).astype(np.float32)

        if raw.size == 0:
            return False
        if raw.shape[1] != 3:
            raw = raw.reshape(-1, 3).astype(np.float32, copy=False)
        else:
            raw = raw.astype(np.float32, copy=False)

        # Random decimation if too dense — protects worst-case clouds
        # (Isaac Sim's 16-line LiDAR running at 20 Hz can spike).
        if raw.shape[0] > self._max_pts_per_cloud:
            idx = np.random.default_rng().choice(
                raw.shape[0], self._max_pts_per_cloud, replace=False,
            )
            raw = raw[idx]

        # Transform points into the map frame (vectorised).
        pts_map = raw @ R.T + translation

        stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        self._frames.append(
            PointCloudClusterAssociator._Frame(
                stamp_ns=stamp_ns, points=pts_map,
            )
        )
        return True

    def latest(self) -> Optional["PointCloudClusterAssociator._Frame"]:
        if not self._frames:
            return None
        return self._frames[-1]

    def associate(
        self,
        x: float,
        y: float,
        *,
        search_radius_m: float,
        z_min: float,
        z_max: float,
        min_cluster_points: int,
        max_cluster_points: int,
        cluster_tolerance_m: float,
        max_aspect_ratio: float = 6.0,
        max_height_m: float = 2.5,
    ) -> PointCloudResult:
        """Find the closest valid cluster to ``(x, y)`` and return it.

        Returns a ``PointCloudResult`` whose ``success=False`` cases
        carry a ``rejection_reason`` mapping 1:1 onto the spec list.
        """
        empty = PointCloudResult(
            success=False, snapped_x=x, snapped_y=y, snapped_z=0.0,
            cluster_id="", point_count=0, bbox_length_m=0.0,
            bbox_width_m=0.0, bbox_height_m=0.0,
            rejection_reason="",
        )
        frame = self.latest()
        if frame is None:
            empty.rejection_reason = "no_pointcloud"
            return empty

        pts = frame.points
        # Class-specific Z slab.
        z_mask = (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
        if not z_mask.any():
            empty.rejection_reason = "no_points_near_detection"
            return empty
        sub = pts[z_mask]

        # Radius slice in XY around the detection.
        dxy = sub[:, :2] - np.array([x, y], dtype=np.float32)
        dist2 = (dxy * dxy).sum(axis=1)
        in_radius = dist2 <= (search_radius_m * search_radius_m)
        if not in_radius.any():
            empty.rejection_reason = "no_points_near_detection"
            return empty
        local = sub[in_radius]
        if local.shape[0] < max(1, int(min_cluster_points)):
            empty.point_count = int(local.shape[0])
            empty.rejection_reason = "cluster_too_small"
            return empty

        # Voxel-grid 4-connected clustering on XY (Z is sliced already).
        # ``tol`` is the voxel size; tuning it is the user's lever for
        # "two close people" vs "one big blob".
        tol = max(0.05, float(cluster_tolerance_m))
        ix = np.floor(local[:, 0] / tol).astype(np.int32)
        iy = np.floor(local[:, 1] / tol).astype(np.int32)
        # Group point indices by voxel for the BFS adjacency lookup.
        voxel_to_idx: Dict[Tuple[int, int], List[int]] = {}
        for i, (cx, cy) in enumerate(zip(ix.tolist(), iy.tolist())):
            voxel_to_idx.setdefault((cx, cy), []).append(i)

        # BFS over voxels. cluster_label[i] = component id of point i.
        labels = np.full(local.shape[0], -1, dtype=np.int32)
        next_id = 0
        # Use a simple iterative deque-based BFS — small voxel counts.
        for vk in voxel_to_idx.keys():
            if labels[voxel_to_idx[vk][0]] != -1:
                continue
            queue: Deque[Tuple[int, int]] = deque([vk])
            comp_id = next_id
            next_id += 1
            for j in voxel_to_idx[vk]:
                labels[j] = comp_id
            while queue:
                cx, cy = queue.popleft()
                for nx, ny in (
                    (cx + 1, cy), (cx - 1, cy),
                    (cx, cy + 1), (cx, cy - 1),
                ):
                    if (nx, ny) not in voxel_to_idx:
                        continue
                    if labels[voxel_to_idx[(nx, ny)][0]] != -1:
                        continue
                    for j in voxel_to_idx[(nx, ny)]:
                        labels[j] = comp_id
                    queue.append((nx, ny))

        # Score each cluster: distance from its centroid to (x,y).
        best_id = -1
        best_d = float("inf")
        best_pts = None
        for cid in range(next_id):
            mask = labels == cid
            n = int(mask.sum())
            if n < max(1, int(min_cluster_points)):
                continue
            cluster_pts = local[mask]
            cx = float(cluster_pts[:, 0].mean())
            cy = float(cluster_pts[:, 1].mean())
            d = (cx - x) ** 2 + (cy - y) ** 2
            if d < best_d:
                best_d = d
                best_id = cid
                best_pts = cluster_pts

        if best_id < 0 or best_pts is None:
            empty.rejection_reason = "cluster_too_small"
            return empty
        n_best = int(best_pts.shape[0])
        if n_best > int(max_cluster_points):
            empty.point_count = n_best
            empty.rejection_reason = "cluster_too_large"
            return empty

        cx = float(best_pts[:, 0].mean())
        cy = float(best_pts[:, 1].mean())
        cz = float(np.median(best_pts[:, 2]))  # median ≈ torso/top
        x_min, x_max = float(best_pts[:, 0].min()), float(best_pts[:, 0].max())
        y_min, y_max = float(best_pts[:, 1].min()), float(best_pts[:, 1].max())
        z_min_b, z_max_b = float(best_pts[:, 2].min()), float(best_pts[:, 2].max())
        L = max(x_max - x_min, y_max - y_min)
        W = min(x_max - x_min, y_max - y_min)
        H = z_max_b - z_min_b

        if H > max_height_m:
            return PointCloudResult(
                success=False, snapped_x=cx, snapped_y=cy, snapped_z=cz,
                cluster_id="", point_count=n_best,
                bbox_length_m=L, bbox_width_m=W, bbox_height_m=H,
                rejection_reason="cluster_height_invalid",
            )
        if max_aspect_ratio > 0.0:
            # Treat a degenerate W (≤ 5 cm) as definitely wall-like;
            # otherwise the aspect L/W can blow up safely.
            ratio = L / max(W, 0.05)
            if ratio > max_aspect_ratio:
                return PointCloudResult(
                    success=False, snapped_x=cx, snapped_y=cy, snapped_z=cz,
                    cluster_id="", point_count=n_best,
                    bbox_length_m=L, bbox_width_m=W, bbox_height_m=H,
                    rejection_reason="cluster_shape_invalid",
                )

        # Stable id keyed on quantised centroid so two consecutive
        # detections that hit the same cluster share the anchor.
        sig_ix = int(round(cx / max(0.05, cluster_tolerance_m)))
        sig_iy = int(round(cy / max(0.05, cluster_tolerance_m)))
        cluster_id = f"pc_{sig_ix:+05d}_{sig_iy:+05d}"
        return PointCloudResult(
            success=True, snapped_x=cx, snapped_y=cy, snapped_z=cz,
            cluster_id=cluster_id, point_count=n_best,
            bbox_length_m=L, bbox_width_m=W, bbox_height_m=H,
            rejection_reason="",
        )


# ---------------------------------------------------------------------------
# Occupancy-island association (Task 1)
# ---------------------------------------------------------------------------
@dataclass
class IslandResult:
    """Outcome of trying to anchor an observation to an obstacle island.

    success:
        True iff a valid (non-wall, non-empty, in-bounds) island was
        found within the search radius and the observation has been
        snapped to its centroid.
    snapped_x / snapped_y:
        World-frame XY of the island centroid (when success=True),
        otherwise the original observation XY echoed back so callers
        can fall through without branching.
    island_id:
        Stable string signature derived from the centroid grid cell.
        Two observations that snap to the same island will share the
        same ``island_id`` even if their raw detections were dozens of
        cm apart, which is what enables "merge by island" logic.
    cell_count:
        Number of occupied cells in the flooded cluster. Useful for
        rejecting tiny noise islands and giant walls.
    bbox_length_m / bbox_width_m:
        Major / minor extent of the cluster's bounding box in metres.
        ``bbox_length / bbox_width`` is the wall-likeness aspect ratio.
    rejection_reason:
        One of:
            ""                          (success)
            occupied_map_unavailable
            outside_map
            unknown_cell
            no_occupied_island_nearby
            island_too_small
            island_too_large
            wall_like_island
            <cls>_island_shape_invalid  (class-specific length / width /
                                         aspect / cell-count limits, e.g.
                                         person_island_shape_invalid)
            <cls>_too_close_to_wall     (class-specific wall-clearance
                                         probe, e.g.
                                         person_too_close_to_wall)
    cells:
        (col, row) pairs for every cell in the island. Filled only
        when ``collect_cells=True`` so the debug marker publisher can
        render the cluster overlay; the hot path passes False to skip
        the allocation.
    """

    success: bool
    snapped_x: float
    snapped_y: float
    island_id: str
    cell_count: int
    bbox_length_m: float
    bbox_width_m: float
    rejection_reason: str
    cells: List[Tuple[int, int]] = field(default_factory=list)


class OccupancyIslandAssociator:
    """Snap an observation to the nearest obstacle island on /map.

    Algorithm (single-component flood from the closest occupied cell):
        1. World→cell using OccupancyGrid origin + resolution.
        2. Reject if the observation cell is outside the grid.
        3. Scan the (search_radius_m)² neighbourhood around the cell,
           collect every occupied (>= occupied_threshold) cell, pick
           the one geometrically closest to the observation.
        4. BFS-flood through 8-connected occupied cells starting from
           that nearest cell. Bounded by ``2 × max_island_cells`` so a
           wall flood-fill can't run away.
        5. Reject the cluster if cell_count is outside the
           [min_island_cells, max_island_cells] range, or its bbox
           aspect ratio + length flag it as a wall.
        6. Compute the centroid in world frame and return it as the
           snapped position.

    No scipy dependency: a hand-written BFS on a small (≤ 40×40) ROI is
    plenty fast enough at <30 Hz detection rates with single-digit
    detection counts per frame.
    """

    def __init__(self) -> None:
        self._latest_map: Optional[OccupancyGrid] = None

    def update_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def has_map(self) -> bool:
        return self._latest_map is not None

    @property
    def latest_map(self) -> Optional[OccupancyGrid]:
        return self._latest_map

    def associate(
        self,
        x: float,
        y: float,
        *,
        search_radius_m: float,
        occupied_threshold: int,
        min_island_cells: int,
        max_island_cells: int,
        reject_wall_like: bool,
        wall_aspect_ratio: float,
        wall_min_length_m: float,
        # ----------------------------------------------------------
        # Class-specific overrides (Task 1 / Task 2). Pass ``None``
        # or 0 to fall back on the class-agnostic defaults above.
        # ``class_name`` is woven into the rejection_reason string
        # ("person_too_close_to_wall" vs the generic
        # "too_close_to_wall") so the operator can tell which class
        # rule fired without re-deriving from context.
        # ----------------------------------------------------------
        class_name: str = "",
        class_min_island_cells: Optional[int] = None,
        class_max_island_cells: Optional[int] = None,
        class_max_island_length_m: Optional[float] = None,
        class_max_island_width_m: Optional[float] = None,
        class_max_island_aspect: Optional[float] = None,
        class_min_wall_clearance_m: float = 0.0,
        wall_clearance_min_external_cells: int = 3,
        wall_clearance_min_long_side_m: float = 0.4,
        collect_cells: bool = False,
    ) -> IslandResult:
        m = self._latest_map
        if m is None:
            return IslandResult(False, x, y, "", 0, 0.0, 0.0,
                                "occupied_map_unavailable")

        info = m.info
        res = float(info.resolution)
        if res <= 0.0:
            return IslandResult(False, x, y, "", 0, 0.0, 0.0,
                                "occupied_map_unavailable")
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        w = int(info.width)
        h = int(info.height)
        data = m.data  # array.array of int8

        cx = int(math.floor((x - ox) / res))
        cy = int(math.floor((y - oy) / res))
        if not (0 <= cx < w and 0 <= cy < h):
            return IslandResult(False, x, y, "", 0, 0.0, 0.0,
                                "outside_map")

        # Centre cell value: -1 = unknown, 0..100 = probability of
        # occupancy. We use it only for diagnostic feedback.
        cv = int(data[cy * w + cx])
        if cv < 0 and cv != -1:
            cv = -1

        # Search ROI in cell units.
        r_cells = max(1, int(math.ceil(search_radius_m / res)))
        x0 = max(0, cx - r_cells)
        x1 = min(w - 1, cx + r_cells)
        y0 = max(0, cy - r_cells)
        y1 = min(h - 1, cy + r_cells)

        # Find the occupied cell closest to (cx, cy) inside the ROI.
        best_dx_dy: Optional[Tuple[int, int]] = None
        best_d2 = (r_cells + 1) ** 2
        for ry in range(y0, y1 + 1):
            row_off = ry * w
            for rx in range(x0, x1 + 1):
                v = int(data[row_off + rx])
                if v < occupied_threshold or v < 0:
                    continue
                ddx = rx - cx
                ddy = ry - cy
                d2 = ddx * ddx + ddy * ddy
                if d2 < best_d2:
                    best_d2 = d2
                    best_dx_dy = (rx, ry)

        if best_dx_dy is None:
            # Special-case: if every cell in the ROI is unknown (-1),
            # report unknown_cell instead of generic
            # no_occupied_island_nearby so the operator can tell that
            # the area simply hasn't been mapped yet.
            any_known = False
            for ry in range(y0, y1 + 1):
                row_off = ry * w
                for rx in range(x0, x1 + 1):
                    if int(data[row_off + rx]) >= 0:
                        any_known = True
                        break
                if any_known:
                    break
            return IslandResult(False, x, y, "", 0, 0.0, 0.0,
                                "unknown_cell" if not any_known
                                else "no_occupied_island_nearby")

        # Flood-fill from the nearest occupied cell. 8-connected.
        cell_budget = max(max_island_cells * 2, 64)
        visited: Dict[Tuple[int, int], bool] = {best_dx_dy: True}
        queue: deque = deque([best_dx_dy])
        cluster: List[Tuple[int, int]] = [best_dx_dy]
        min_cx = max_cx = best_dx_dy[0]
        min_cy = max_cy = best_dx_dy[1]
        while queue:
            qx, qy = queue.popleft()
            for ddx in (-1, 0, 1):
                for ddy in (-1, 0, 1):
                    if ddx == 0 and ddy == 0:
                        continue
                    nx = qx + ddx
                    ny = qy + ddy
                    if not (x0 <= nx <= x1 and y0 <= ny <= y1):
                        continue
                    key = (nx, ny)
                    if key in visited:
                        continue
                    v = int(data[ny * w + nx])
                    if v < occupied_threshold or v < 0:
                        continue
                    visited[key] = True
                    cluster.append(key)
                    if nx < min_cx:
                        min_cx = nx
                    elif nx > max_cx:
                        max_cx = nx
                    if ny < min_cy:
                        min_cy = ny
                    elif ny > max_cy:
                        max_cy = ny
                    if len(cluster) >= cell_budget:
                        # Treat budget exhaustion as wall-like.
                        return IslandResult(
                            False, x, y, "", len(cluster), 0.0, 0.0,
                            "island_too_large",
                            cells=cluster if collect_cells else [])
                    queue.append(key)

        n_cells = len(cluster)
        # Class-specific minimum cells trumps the global one when set.
        eff_min_cells = (
            int(class_min_island_cells)
            if class_min_island_cells is not None
            and class_min_island_cells > 0
            else min_island_cells
        )
        if n_cells < eff_min_cells:
            return IslandResult(False, x, y, "", n_cells, 0.0, 0.0,
                                "island_too_small",
                                cells=cluster if collect_cells else [])
        if n_cells > max_island_cells:
            return IslandResult(False, x, y, "", n_cells, 0.0, 0.0,
                                "island_too_large",
                                cells=cluster if collect_cells else [])

        bbox_w_cells = (max_cx - min_cx) + 1
        bbox_h_cells = (max_cy - min_cy) + 1
        long_side = max(bbox_w_cells, bbox_h_cells) * res
        short_side = max(min(bbox_w_cells, bbox_h_cells) * res, res)
        aspect = long_side / short_side
        if reject_wall_like and long_side >= wall_min_length_m \
                and aspect >= wall_aspect_ratio:
            return IslandResult(False, x, y, "", n_cells,
                                long_side, short_side,
                                "wall_like_island",
                                cells=cluster if collect_cells else [])

        # ----------------------------------------------------------
        # Task 2 — class-specific shape filter (post wall-aspect).
        # A "person" cluster bigger than ~1.0 m on a side is almost
        # certainly a wall fragment, even if it didn't trip the
        # generic wall_like check.
        # ----------------------------------------------------------
        cls_tag = (class_name or "").strip().lower()
        shape_reason = (
            f"{cls_tag}_island_shape_invalid"
            if cls_tag else "island_shape_invalid"
        )
        if (
            class_max_island_cells is not None
            and class_max_island_cells > 0
            and n_cells > class_max_island_cells
        ):
            return IslandResult(False, x, y, "", n_cells,
                                long_side, short_side,
                                shape_reason,
                                cells=cluster if collect_cells else [])
        if (
            class_max_island_length_m is not None
            and class_max_island_length_m > 0.0
            and long_side > class_max_island_length_m
        ):
            return IslandResult(False, x, y, "", n_cells,
                                long_side, short_side,
                                shape_reason,
                                cells=cluster if collect_cells else [])
        if (
            class_max_island_width_m is not None
            and class_max_island_width_m > 0.0
            and short_side > class_max_island_width_m
        ):
            return IslandResult(False, x, y, "", n_cells,
                                long_side, short_side,
                                shape_reason,
                                cells=cluster if collect_cells else [])
        if (
            class_max_island_aspect is not None
            and class_max_island_aspect > 0.0
            and aspect > class_max_island_aspect
        ):
            return IslandResult(False, x, y, "", n_cells,
                                long_side, short_side,
                                shape_reason,
                                cells=cluster if collect_cells else [])

        # Centroid — mean of cell indices, rounded back to world frame.
        sum_cx = sum(c[0] for c in cluster)
        sum_cy = sum(c[1] for c in cluster)
        mean_cx = sum_cx / n_cells
        mean_cy = sum_cy / n_cells
        snapped_x = ox + (mean_cx + 0.5) * res
        snapped_y = oy + (mean_cy + 0.5) * res

        # ----------------------------------------------------------
        # Task 1 — wall-proximity probe. After we have a candidate
        # island centroid, look at occupied cells in the wider
        # neighbourhood that are NOT part of the cluster. If those
        # external cells form a long span (≥ wall_clearance_min_long_
        # side_m) within ``class_min_wall_clearance_m`` of the snapped
        # point, we're snapping a person observation onto a wall
        # fragment that happens to look small in isolation.
        # ----------------------------------------------------------
        if class_min_wall_clearance_m > 0.0:
            clr_cells = max(1, int(math.ceil(
                class_min_wall_clearance_m / res
            )))
            cluster_set = set(cluster)
            mean_ix = int(round(mean_cx))
            mean_iy = int(round(mean_cy))
            qx0 = max(0, mean_ix - clr_cells)
            qx1 = min(w - 1, mean_ix + clr_cells)
            qy0 = max(0, mean_iy - clr_cells)
            qy1 = min(h - 1, mean_iy + clr_cells)
            ext_count = 0
            ext_min_x = ext_min_y = 1 << 30
            ext_max_x = ext_max_y = -(1 << 30)
            clr_cells2 = clr_cells * clr_cells
            for ry in range(qy0, qy1 + 1):
                row_off = ry * w
                ddy = ry - mean_iy
                for rx in range(qx0, qx1 + 1):
                    if (rx, ry) in cluster_set:
                        continue
                    v = int(data[row_off + rx])
                    if v < occupied_threshold or v < 0:
                        continue
                    ddx = rx - mean_ix
                    if ddx * ddx + ddy * ddy > clr_cells2:
                        continue
                    ext_count += 1
                    if rx < ext_min_x:
                        ext_min_x = rx
                    if rx > ext_max_x:
                        ext_max_x = rx
                    if ry < ext_min_y:
                        ext_min_y = ry
                    if ry > ext_max_y:
                        ext_max_y = ry
            if ext_count >= max(1, wall_clearance_min_external_cells):
                ext_long_side = (
                    max(ext_max_x - ext_min_x,
                        ext_max_y - ext_min_y) + 1
                ) * res
                if ext_long_side >= wall_clearance_min_long_side_m:
                    reason = (
                        f"{cls_tag}_too_close_to_wall"
                        if cls_tag else "too_close_to_wall"
                    )
                    return IslandResult(False, x, y, "", n_cells,
                                        long_side, short_side,
                                        reason,
                                        cells=cluster
                                        if collect_cells else [])

        # Stable signature: round to 0.1 m grid so trivially close
        # observations always hash to the same id.
        sig_ix = int(round(snapped_x * 10.0))
        sig_iy = int(round(snapped_y * 10.0))
        island_id = f"isl_{sig_ix:+05d}_{sig_iy:+05d}"

        return IslandResult(
            True, snapped_x, snapped_y, island_id,
            n_cells, long_side, short_side, "",
            cells=cluster if collect_cells else [],
        )


def _class_to_color(class_id: str) -> Tuple[float, float, float]:
    """Deterministic class -> distinct colour mapping for RViz markers.

    Hash the class name to a 24-bit value and split into RGB. Tweaks
    keep the colour saturated and reasonably distinct for the typical
    indoor classes (chair / table / box / ...) without needing a
    hand-curated palette.
    """
    h = hash(class_id) & 0xFFFFFF
    r = ((h >> 16) & 0xFF) / 255.0
    g = ((h >> 8) & 0xFF) / 255.0
    b = (h & 0xFF) / 255.0
    # Push saturation up: scale by max(r,g,b) to avoid muddy mid-greys.
    m = max(r, g, b, 1e-6)
    return (r / m, g / m, b / m)


@dataclass
class TrackedEntity:
    """Mutable in-memory representation of a SemanticEntity row.

    Day 8+ additions
    ----------------
    raw_class:
        The original detector label (e.g. ``"desk"`` even though
        ``class_label`` was canonicalised to ``"table"``). Stored
        purely for debugging / RViz display; never used for matching.
    island_id:
        Stable signature of the obstacle island this entity is anchored
        to (see OccupancyIslandAssociator). ``""`` for un-anchored
        candidates that pre-date a /map message.
    is_confirmed:
        True once the entity has cleared *any* of the promotion paths
        (count threshold, island-anchor + confidence, repeated island
        co-observation). Confirmed entities are persistent — they
        never decay below ``confirmed_min_confidence`` and never get
        pruned, so they survive long traverses and occlusions.
    same_island_observations:
        Counter for "this same class re-observed near the same
        island" events. Provides a third promotion path independent
        of the confidence-step-up curve.
    invalid_evidence_count:
        Day 8++ — running tally of "this entity got matched with a
        rejected island association (wall_like / too_close_to_wall /
        shape_invalid / outside_map / unknown_cell)". Lets the slow
        path retire a confirmed false positive without removing all
        confirmed-landmark persistence.
    is_invalid:
        True once ``invalid_evidence_count`` clears the threshold.
        target_selector skips invalid entities; markers are rerouted
        from /semantic_map/markers (confirmed) to /semantic_map/
        debug_markers (candidate / debug) so they remain visible for
        the operator.
    candidate_status_hint:
        When ``is_confirmed`` is False and there is no anchor, encodes
        why the entity stays a candidate (``wall_like_island`` /
        ``near_unknown`` / ``no_anchor``). Cleared once anchored or
        confirmed. Surfaced in RViz text as
        ``candidate_not_confirmed: <hint>``.
    """

    entity_id: str
    class_label: str
    px: float
    py: float
    pz: float
    confidence: float
    observations_count: int
    first_seen: Time
    last_seen: Time
    currently_visible: bool = True
    raw_class: str = ""
    island_id: str = ""
    is_confirmed: bool = False
    same_island_observations: int = 0
    invalid_evidence_count: int = 0
    is_invalid: bool = False
    candidate_status_hint: str = ""


class SemanticMemoryAggregatorNode(Node):
    """Persistent object aggregator with NMS + confidence decay."""

    def __init__(self) -> None:
        super().__init__("semantic_memory_aggregator")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------
        self.declare_parameter(
            "detections_3d_topic", "/detections_3d"
        )
        self.declare_parameter(
            "objects_topic", "/semantic_map/objects"
        )
        self.declare_parameter(
            "markers_topic", "/semantic_map/markers"
        )
        # Day 8+: RViz consumes ``markers_topic`` = **confirmed** landmarks
        # only (visible + remembered in one layer). Candidate / invalid /
        # rejected-no-anchor stream lives on ``debug_markers_topic``. Optional:
        # ``markers_visible`` / ``markers_remembered`` duplicate confirmed
        # markers by ``currently_visible`` for cleaner demo overlays.
        self.declare_parameter(
            "debug_markers_topic", "/semantic_map/debug_markers"
        )
        self.declare_parameter(
            "publish_split_visibility_markers", True
        )
        self.declare_parameter(
            "visible_markers_topic",
            "/semantic_map/markers_visible",
        )
        self.declare_parameter(
            "remembered_markers_topic",
            "/semantic_map/markers_remembered",
        )
        self.declare_parameter(
            "island_debug_markers_topic",
            "/semantic_map/island_debug_markers",
        )
        # OccupancyGrid input for island association (Day 8+).
        # Defaults to /map from slam_toolbox; switch to a costmap topic
        # when running with a static map + AMCL.
        self.declare_parameter("map_topic", "/map")
        # Operator hook: publish a String on this topic to manipulate
        # the entity registry at runtime without restarting the node.
        # Supported commands (case-insensitive):
        #   "reset" / "clear" / "clear_all"
        #       Drop ALL entities, including permanent landmarks.
        #       Use after a stale-TF window glues spurious chairs to
        #       the wall.
        #   "clear_non_permanent" / "clear_transient"
        #       Drop only entities below permanent_after_observations.
        #       Keeps trusted landmarks; sweeps out flicker-detection
        #       junk from the start of a Phase A run.
        # Anything else is logged as a warn and ignored.
        self.declare_parameter(
            "control_topic", "/semantic_map/control"
        )
        # Spatial NMS radius (metres) for matching a fresh detection
        # to an existing entity of the same class. 0.3 m is large
        # enough to absorb depth-median jitter on a static chair,
        # small enough to keep two real chairs at 1 m apart distinct.
        self.declare_parameter("nms_radius_m", 0.3)
        # Exponential moving average weight for fresh observation
        # vs prior estimate (alpha=1.0 -> instant overwrite,
        # alpha=0.0 -> never update).
        self.declare_parameter("position_alpha", 0.3)
        # Confidence increment per matched observation, additive.
        # The cap is at 1.0; combined with `confidence_decay_rate`
        # below this gives a saturation behaviour where a stably-
        # observed object hovers near 1.0 and a missing object
        # decays exponentially.
        self.declare_parameter("confidence_step_up", 0.15)
        # Age-aware exponential decay rate. On each housekeeping tick
        # we apply ``confidence *= exp(-decay_rate * age_since_last_seen)``,
        # so an entity that hasn't been observed for a long time
        # decays MORE per tick than a recently-observed one. The
        # alternative (a fixed multiplicative decay every tick,
        # regardless of age) double-decays entities that already
        # haven't been seen — the Day 6 plan explicitly calls this
        # out as the right way to do it. Default 0.05 yields a
        # half-life of ln(2)/0.05 ≈ 13.9 s.
        self.declare_parameter("confidence_decay_rate", 0.05)
        # Drop fresh detections whose hypothesis.score is below this
        # threshold BEFORE feeding the aggregator. This is upstream of
        # NMS and keeps a single 0.15-score false positive from ever
        # creating an entity (which would then take many seconds to
        # decay away). Day 5 YOLOE node already filters at
        # conf_threshold (0.4 default), so by the time detections
        # reach Day 6 they're typically >0.4 — this is a second-line
        # defence for low-quality detectors. Set 0 to disable.
        self.declare_parameter("min_detection_confidence", 0.4)
        # Relaxed per-class floors for *admitting* person/table
        # detections into the candidate pool (does not disable
        # ``min_detection_confidence`` for other classes). Confirmed
        # promotion still uses ``confirmed_min_confidence`` and anchor
        # gates.
        self.declare_parameter(
            "candidate_min_confidence_person", 0.35
        )
        self.declare_parameter(
            "candidate_min_confidence_table", 0.35
        )
        # When False, /semantic_map/debug_markers omits candidate
        # cylinders (DELETEALL only). Confirmed + invalid still follow
        # their normal routing.
        self.declare_parameter("publish_candidates", True)
        # If an entity hasn't been matched in this many seconds,
        # mark currently_visible = False (but keep it in memory).
        self.declare_parameter("visibility_timeout_sec", 2.0)
        # Final pruning thresholds: entity is dropped only when
        # BOTH conditions hold (low confidence AND stale).
        self.declare_parameter("prune_confidence_threshold", 0.05)
        self.declare_parameter("prune_age_sec", 30.0)
        # Once an entity has been observed this many times, freeze it
        # as a *permanent / confirmed landmark*: no more confidence
        # decay and no pruning. Day 8+: lowered the default from 5 to
        # 2 because the operator-level pain point is "Go2 sees the
        # person for half a second, looks away, the person marker is
        # gone". With island association (below), 2 confident
        # observations on the same island are usually enough to trust.
        self.declare_parameter("permanent_after_observations", 2)
        # Same-class post-association merge radius (metres) — legacy
        # default. Class-specific overrides below win when set.
        self.declare_parameter("entity_merge_radius_m", 1.5)
        # Class-specific merge radii. The aggregator picks the value
        # for the entity's CANONICAL class; falls back to
        # ``entity_merge_radius_m`` if no entry matches.
        # Demo stability: slightly larger radii so duplicate person/table
        # markers converge within a few housekeeping cycles.
        self.declare_parameter("merge_person_radius_m", 1.5)
        self.declare_parameter("merge_table_radius_m", 2.5)
        self.declare_parameter("merge_chair_radius_m", 0.6)
        # When True, two entities sharing a non-empty ``island_id``
        # get merged regardless of the geometric merge radius. This
        # is the strongest deduplication signal: no matter how far
        # apart YOLOE jitter pushes the two detections, if they both
        # snap to the same obstacle cluster they are the same object.
        self.declare_parameter("merge_by_island_id", True)
        # MVP single-instance cap. When >0, only the top-N "best"
        # confirmed entities of the named class survive each
        # housekeeping pass; the rest are demoted (is_confirmed=False
        # so they age out via the normal candidate decay) so the
        # /semantic_map/markers stream cannot grow stale duplicates.
        # Day 8++ default: ONE confirmed person — the demo only has
        # one. Day 9 (table-cleanup): tighten table to 1 too — the
        # warehouse has exactly one real table, but we routinely saw
        # 3-4 confirmed table markers because YOLOE fires "table" /
        # "desk" / "dining table" from oblique angles and the
        # depth-projected centroids land outside the same merge
        # radius. Capping to 1 makes the /semantic_map/markers
        # stream the ground truth.
        self.declare_parameter("max_confirmed_person_landmarks", 1)
        self.declare_parameter("max_confirmed_table_landmarks", 1)
        # ----------------------------------------------------------
        # Occupancy-island association (Task 1, Day 8+)
        # ----------------------------------------------------------
        self.declare_parameter("use_occupancy_island_association", True)
        self.declare_parameter("island_search_radius_m", 1.0)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("min_island_cells", 2)
        self.declare_parameter("max_island_cells", 800)
        self.declare_parameter("reject_wall_like_islands", True)
        self.declare_parameter("wall_like_aspect_ratio", 6.0)
        self.declare_parameter("wall_like_min_length_m", 2.0)
        # When True, observations that fail island association are
        # still registered as low-confidence candidates so they show
        # up in /semantic_map/debug_markers — useful for diagnosing
        # "why did my detection get rejected?". Set False for strict
        # island-only mode.
        self.declare_parameter("keep_unanchored_candidates", True)
        # ----------------------------------------------------------
        # Class-specific island shape + wall-clearance constraints
        # (Tasks 1 + 2). Used to reject "person snapped to a wall
        # fragment" without over-constraining tables, which can
        # legitimately sit close to walls.
        # 0 / negative ⇒ disable that particular constraint.
        # ----------------------------------------------------------
        self.declare_parameter("reject_person_near_wall", True)
        self.declare_parameter("person_min_island_cells", 3)
        self.declare_parameter("person_max_island_cells", 250)
        self.declare_parameter("person_max_island_length_m", 1.0)
        self.declare_parameter("person_max_island_width_m", 1.0)
        self.declare_parameter("person_max_island_aspect_ratio", 3.0)
        self.declare_parameter("person_min_wall_clearance_m", 0.35)
        self.declare_parameter("table_min_island_cells", 2)
        self.declare_parameter("table_max_island_cells", 800)
        self.declare_parameter("table_max_island_length_m", 2.2)
        self.declare_parameter("table_max_island_width_m", 1.6)
        self.declare_parameter("table_max_island_aspect_ratio", 4.0)
        self.declare_parameter("table_min_wall_clearance_m", 0.15)
        # Wall-clearance probe knobs (apply to whichever class is
        # active; the long-side threshold is what makes a wall vs
        # noise). ``min_external_cells`` adds a noise floor: a
        # single straggler occupied cell from a phantom rock should
        # NOT trigger person_too_close_to_wall.
        self.declare_parameter(
            "wall_clearance_min_external_cells", 3
        )
        self.declare_parameter(
            "wall_clearance_min_long_side_m", 0.4
        )
        # ----------------------------------------------------------
        # Confirmed-landmark invalidation (Task 3). Confirmed entities
        # never decay/prune, but they CAN accumulate "this association
        # tried to anchor me on a wall again" evidence. After enough
        # such hits we tag the entity ``is_invalid=True`` so the
        # target selector skips it; the marker is rerouted to the
        # debug-only stream so RViz still shows the operator that
        # something was once confirmed there but has been rejected.
        # ----------------------------------------------------------
        self.declare_parameter("allow_confirmed_invalidation", True)
        self.declare_parameter("confirmed_invalid_evidence_threshold", 3)
        # ----------------------------------------------------------
        # Persistent confirmed landmarks (Task 2, Day 8+)
        # ----------------------------------------------------------
        self.declare_parameter("keep_confirmed_landmarks", True)
        self.declare_parameter("confirmed_min_confidence", 0.5)
        # Promotion path 2: confidence threshold to promote an
        # island-anchored candidate to confirmed *immediately* (i.e.
        # without waiting for ``permanent_after_observations``). Set
        # >1.0 to disable this fast-path.
        self.declare_parameter("island_promotion_confidence", 0.5)
        # Promotion path 3: same class observed near the same island
        # this many times. Default 2 means "twice on the same island"
        # is enough; lowering to 1 makes any island-anchored
        # observation confirm immediately.
        self.declare_parameter("island_promotion_count", 2)
        # ----------------------------------------------------------
        # Day 8++ — per-class promotion gates (Task 4)
        #
        # The Day 8 fast-path "single high-confidence detection +
        # island anchor ⇒ confirmed" was too aggressive for person:
        # YOLOE happily fires score=0.95 on a window reflection or a
        # painted-on poster, and once that lands inside an island
        # search radius we'd ship a confirmed person for navigation.
        # The fix: PER-CLASS minimum observations + an explicit
        # opt-in for the single-observation island fast-path.
        #
        #   person_min_observations_to_confirm  >= 2 by default
        #   table_min_observations_to_confirm   = 1 (table mostly
        #     stays stable across frames; we still benefit from
        #     fast-confirming a furniture detection)
        #   allow_single_observation_island_promotion_classes
        #     space-separated allow-list. Empty by default ⇒ NO class
        #     gets the single-obs fast-path. "table" allows it for
        #     table only.
        # ----------------------------------------------------------
        self.declare_parameter("person_min_observations_to_confirm", 2)
        # Stricter remembered landmarks for furniture: require at
        # least two qualifying observations unless the island fast-path
        # already satisfied ``class_min_obs`` via recurrence.
        self.declare_parameter("table_min_observations_to_confirm", 2)
        # Empty default — table now uses ``table_min_observations_to_confirm``
        # so "single frame desk" does not jump straight to confirmed.
        self.declare_parameter(
            "allow_single_observation_island_promotion_classes", ""
        )
        # When True, a single high-quality ``pc_*`` snap for ``table`` may
        # confirm immediately (default False — require >=2 observations).
        self.declare_parameter(
            "table_allow_single_pc_anchor_promotion", False,
        )
        # ----------------------------------------------------------
        # Day 8++ — publication gating (Task 5).
        # Classes named here MUST have a non-empty island_id before
        # they are emitted on /semantic_map/markers (the "confirmed"
        # marker stream consumed by RViz + target_selector). Confirmed
        # entities without an anchor are still emitted on the
        # debug stream so the operator can see them, but they are NOT
        # selectable as navigation targets. The matching filter on
        # the target_selector side carries the same default.
        # ``island_id`` includes ``pc_*`` (LiDAR cluster) and ``isl_*``
        # (/map obstacle island shape) — both satisfy the gate.
        self.declare_parameter(
            "require_island_anchor_for_classes", "person,table"
        )
        # Day 8++++ Task 3 — when a *confirmed* entity of one of the
        # ``require_island_anchor_for_classes`` classes is later found
        # to have no island anchor (display_name "...|-"), should we
        # demote (False) or mark invalid (True)?
        # Demote: clears is_confirmed; entity continues as a candidate
        # and decays normally. Mark invalid: tags is_invalid=True so
        # the entity is rerouted to the INVALID red marker stream and
        # the operator can see the false positive.
        self.declare_parameter(
            "mark_unanchored_required_classes_invalid", True,
        )
        # ----------------------------------------------------------
        # Day 9 Task 1 — PointCloud2 cluster anchoring
        # ----------------------------------------------------------
        # Why this exists: /map obstacle islands are unreliable for
        # tables (thin legs reflect LiDAR badly) and people (sliver of
        # occupied cells unless robot translates). /lidar/points has
        # the geometry from frame one — use it as the *primary*
        # anchor, demote occupancy islands to cross-validation.
        self.declare_parameter("use_pointcloud_cluster_anchor", True)
        self.declare_parameter("pointcloud_topic", "/lidar/points")
        self.declare_parameter("pointcloud_anchor_search_radius_m", 1.0)
        self.declare_parameter("person_pointcloud_search_radius_m", 1.2)
        self.declare_parameter("table_pointcloud_search_radius_m", 1.5)
        self.declare_parameter("pointcloud_min_cluster_points", 5)
        self.declare_parameter("pointcloud_max_cluster_points", 5000)
        self.declare_parameter("pointcloud_cluster_tolerance_m", 0.20)
        # Class-specific Z slabs cut down on floor + ceiling clutter
        # before clustering. Person torso is ~1m, head ~1.7m → keep
        # 0.05–1.9. Table top ~0.7m, but legs go to floor → keep
        # 0.05–1.3 so the table top contributes to the centroid Z.
        self.declare_parameter("person_pointcloud_z_min", 0.05)
        self.declare_parameter("person_pointcloud_z_max", 1.9)
        self.declare_parameter("table_pointcloud_z_min", 0.05)
        self.declare_parameter("table_pointcloud_z_max", 1.3)
        self.declare_parameter("pointcloud_buffer_size", 4)
        self.declare_parameter("pointcloud_max_points_per_cloud", 20000)
        # Day 9 Task 5 — anchor debug stats publication.
        self.declare_parameter(
            "anchor_debug_stats_topic",
            "/semantic_map/anchor_debug_stats",
        )
        self.declare_parameter("anchor_debug_stats_period_sec", 2.0)
        # ----------------------------------------------------------
        # Canonical class map (Task 4, Day 8+)
        # Format: list of "alias=canonical" strings, e.g.
        #   ["desk=table", "workbench=table", "worker=person"]
        # Empty list ⇒ use the built-in defaults
        # (_DEFAULT_CANONICAL_CLASS_MAP).
        # ----------------------------------------------------------
        self.declare_parameter("canonical_class_map", [""])
        # Housekeeping tick period.
        self.declare_parameter("housekeeping_period_sec", 1.0)
        # Frame for publishing markers + entity poses. Should match
        # depth_projector's `target_frame` (default "map").
        self.declare_parameter("frame_id", "map")
        # FPS log heartbeat.
        self.declare_parameter("log_period_sec", 5.0)

        det_topic = str(self.get_parameter("detections_3d_topic").value)
        obj_topic = str(self.get_parameter("objects_topic").value)
        mk_topic = str(self.get_parameter("markers_topic").value)
        debug_mk_topic = str(
            self.get_parameter("debug_markers_topic").value
        )
        island_dbg_topic = str(
            self.get_parameter("island_debug_markers_topic").value
        )
        self._publish_split_visibility_markers = bool(
            self.get_parameter("publish_split_visibility_markers").value
        )
        self._visible_split_mk_topic = str(
            self.get_parameter("visible_markers_topic").value or ""
        )
        self._remembered_split_mk_topic = str(
            self.get_parameter("remembered_markers_topic").value or ""
        )
        map_topic = str(self.get_parameter("map_topic").value)
        self._nms_r2 = float(self.get_parameter("nms_radius_m").value) ** 2
        self._alpha = float(self.get_parameter("position_alpha").value)
        self._conf_up = float(self.get_parameter("confidence_step_up").value)
        self._conf_decay_rate = float(
            self.get_parameter("confidence_decay_rate").value
        )
        self._min_det_conf = float(
            self.get_parameter("min_detection_confidence").value
        )
        self._cand_min_person = float(
            self.get_parameter("candidate_min_confidence_person").value
        )
        self._cand_min_table = float(
            self.get_parameter("candidate_min_confidence_table").value
        )
        self._publish_candidates = bool(
            self.get_parameter("publish_candidates").value
        )
        self._vis_timeout = float(
            self.get_parameter("visibility_timeout_sec").value
        )
        self._prune_conf = float(
            self.get_parameter("prune_confidence_threshold").value
        )
        self._prune_age = float(self.get_parameter("prune_age_sec").value)
        self._permanent_n = int(
            self.get_parameter("permanent_after_observations").value
        )
        merge_r = float(
            self.get_parameter("entity_merge_radius_m").value
        )
        self._merge_r2 = merge_r * merge_r if merge_r > 0.0 else 0.0
        self._merge_radius_by_class: Dict[str, float] = {
            "person": float(
                self.get_parameter("merge_person_radius_m").value
            ),
            "table": float(
                self.get_parameter("merge_table_radius_m").value
            ),
            "chair": float(
                self.get_parameter("merge_chair_radius_m").value
            ),
        }
        self._merge_by_island_id = bool(
            self.get_parameter("merge_by_island_id").value
        )
        # Island association params
        self._use_island = bool(
            self.get_parameter("use_occupancy_island_association").value
        )
        self._island_radius = float(
            self.get_parameter("island_search_radius_m").value
        )
        self._occ_thresh = int(
            self.get_parameter("occupied_threshold").value
        )
        self._island_min = int(
            self.get_parameter("min_island_cells").value
        )
        self._island_max = int(
            self.get_parameter("max_island_cells").value
        )
        self._reject_walls = bool(
            self.get_parameter("reject_wall_like_islands").value
        )
        self._wall_aspect = float(
            self.get_parameter("wall_like_aspect_ratio").value
        )
        self._wall_min_len = float(
            self.get_parameter("wall_like_min_length_m").value
        )
        self._keep_unanchored = bool(
            self.get_parameter("keep_unanchored_candidates").value
        )
        # Class-specific island shape + wall-clearance overrides.
        self._reject_person_near_wall = bool(
            self.get_parameter("reject_person_near_wall").value
        )
        self._wall_clear_min_ext = int(
            self.get_parameter("wall_clearance_min_external_cells").value
        )
        self._wall_clear_min_long = float(
            self.get_parameter("wall_clearance_min_long_side_m").value
        )

        def _f(name: str) -> float:
            return float(self.get_parameter(name).value)

        def _i(name: str) -> int:
            return int(self.get_parameter(name).value)

        # ``_class_constraints`` lookup keyed by canonical class. Each
        # entry is a dict the associator pulls into kwargs. Missing
        # classes (e.g. "box") fall through to the global defaults.
        self._class_constraints: Dict[str, Dict[str, float]] = {
            "person": {
                "min_cells": _i("person_min_island_cells"),
                "max_cells": _i("person_max_island_cells"),
                "max_length_m": _f("person_max_island_length_m"),
                "max_width_m": _f("person_max_island_width_m"),
                "max_aspect": _f("person_max_island_aspect_ratio"),
                "min_wall_clearance_m": (
                    _f("person_min_wall_clearance_m")
                    if self._reject_person_near_wall else 0.0
                ),
            },
            "table": {
                "min_cells": _i("table_min_island_cells"),
                "max_cells": _i("table_max_island_cells"),
                "max_length_m": _f("table_max_island_length_m"),
                "max_width_m": _f("table_max_island_width_m"),
                "max_aspect": _f("table_max_island_aspect_ratio"),
                "min_wall_clearance_m": _f("table_min_wall_clearance_m"),
            },
        }
        # Confirmed-entity invalidation knobs.
        self._allow_invalid = bool(
            self.get_parameter("allow_confirmed_invalidation").value
        )
        self._invalid_thresh = int(
            self.get_parameter("confirmed_invalid_evidence_threshold").value
        )
        # Frozenset for cheap membership checks in the hot path.
        self._invalidating_reasons = frozenset({
            "wall_like_island",
            "outside_map",
            "unknown_cell",
            "person_too_close_to_wall",
            "table_too_close_to_wall",
            "too_close_to_wall",
            "person_island_shape_invalid",
            "table_island_shape_invalid",
            "island_shape_invalid",
        })
        # Persistent confirmed landmark params
        self._keep_confirmed = bool(
            self.get_parameter("keep_confirmed_landmarks").value
        )
        self._confirmed_min_conf = float(
            self.get_parameter("confirmed_min_confidence").value
        )
        self._island_promo_conf = float(
            self.get_parameter("island_promotion_confidence").value
        )
        self._island_promo_count = int(
            self.get_parameter("island_promotion_count").value
        )
        # Per-class promotion gates (Task 4).
        self._min_obs_to_confirm: Dict[str, int] = {
            "person": int(
                self.get_parameter(
                    "person_min_observations_to_confirm"
                ).value
            ),
            "table": int(
                self.get_parameter(
                    "table_min_observations_to_confirm"
                ).value
            ),
        }
        single_obs_allow_raw = str(
            self.get_parameter(
                "allow_single_observation_island_promotion_classes"
            ).value or ""
        )
        # space-, comma- or semicolon-separated; empty entries filtered.
        self._single_obs_island_allow: set = {
            _normalise_class_key(s)
            for s in single_obs_allow_raw.replace(",", " ").replace(";", " ").split()
            if s.strip()
        }
        self._table_allow_single_pc = bool(
            self.get_parameter(
                "table_allow_single_pc_anchor_promotion"
            ).value,
        )
        # Publication gate (Task 5).
        require_anchor_raw = str(
            self.get_parameter(
                "require_island_anchor_for_classes"
            ).value or ""
        )
        self._require_island_classes: set = {
            _normalise_class_key(s)
            for s in require_anchor_raw.replace(",", " ").replace(";", " ").split()
            if s.strip()
        }
        self._mark_unanchored_invalid = bool(
            self.get_parameter(
                "mark_unanchored_required_classes_invalid"
            ).value
        )
        # Day 9 — pointcloud anchoring.
        self._use_pc_anchor = bool(
            self.get_parameter("use_pointcloud_cluster_anchor").value
        )
        self._pc_topic = str(
            self.get_parameter("pointcloud_topic").value or ""
        )
        self._pc_default_radius = float(
            self.get_parameter("pointcloud_anchor_search_radius_m").value
        )
        self._pc_radius_by_class: Dict[str, float] = {
            "person": float(
                self.get_parameter(
                    "person_pointcloud_search_radius_m"
                ).value
            ),
            "table": float(
                self.get_parameter(
                    "table_pointcloud_search_radius_m"
                ).value
            ),
        }
        self._pc_min_pts = int(
            self.get_parameter("pointcloud_min_cluster_points").value
        )
        self._pc_max_pts = int(
            self.get_parameter("pointcloud_max_cluster_points").value
        )
        self._pc_tol = float(
            self.get_parameter("pointcloud_cluster_tolerance_m").value
        )
        self._pc_z_by_class: Dict[str, Tuple[float, float]] = {
            "person": (
                float(self.get_parameter("person_pointcloud_z_min").value),
                float(self.get_parameter("person_pointcloud_z_max").value),
            ),
            "table": (
                float(self.get_parameter("table_pointcloud_z_min").value),
                float(self.get_parameter("table_pointcloud_z_max").value),
            ),
        }
        self._pc_buffer_size = int(
            self.get_parameter("pointcloud_buffer_size").value
        )
        self._pc_max_pts_per_cloud = int(
            self.get_parameter("pointcloud_max_points_per_cloud").value
        )
        self._anchor_stats_topic = str(
            self.get_parameter("anchor_debug_stats_topic").value or ""
        )
        self._anchor_stats_period = float(
            self.get_parameter("anchor_debug_stats_period_sec").value
        )
        # Per-class confirmed cap (Task 3).
        self._max_confirmed_per_class: Dict[str, int] = {
            "person": int(
                self.get_parameter("max_confirmed_person_landmarks").value
            ),
            "table": int(
                self.get_parameter("max_confirmed_table_landmarks").value
            ),
        }
        # Canonical class map
        raw_map = list(self.get_parameter("canonical_class_map").value or [])
        self._canonical_map: Dict[str, str] = dict(_DEFAULT_CANONICAL_CLASS_MAP)
        for spec in raw_map:
            if not spec or "=" not in spec:
                continue
            alias, canonical = spec.split("=", 1)
            alias = _normalise_class_key(alias)
            canonical = _normalise_class_key(canonical)
            if alias and canonical:
                self._canonical_map[alias] = canonical
        hk_period = float(
            self.get_parameter("housekeeping_period_sec").value
        )
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._log_period = float(self.get_parameter("log_period_sec").value)

        # Validate ranges that would silently break the aggregator.
        if not (0.0 < self._alpha <= 1.0):
            self.get_logger().warn(
                f"position_alpha={self._alpha} outside (0,1]; clamping to 0.3"
            )
            self._alpha = 0.3
        if self._conf_decay_rate < 0.0:
            self.get_logger().warn(
                f"confidence_decay_rate={self._conf_decay_rate} negative; "
                f"clamping to 0.05"
            )
            self._conf_decay_rate = 0.05

        # --------------------------------------------------------------
        # State
        # --------------------------------------------------------------
        self._entities: Dict[str, TrackedEntity] = {}
        # Per-class monotonic counter for human-readable entity ids
        # (`chair_001`, `chair_002`, `table_001`, ...). Reusing the
        # same counter across the lifetime of the node means an
        # object that gets pruned and re-observed gets a *new* id
        # rather than recycling the old one — much less confusing
        # for an operator watching RViz than reusing 'chair_001'
        # for a different chair later.
        self._next_id_per_class: Dict[str, int] = {}
        self._n_messages = 0
        self._n_associations = 0
        self._n_new_entities = 0
        self._last_log_time = self.get_clock().now()

        # --------------------------------------------------------------
        # ROS infra
        # --------------------------------------------------------------
        # /detections_3d is RELIABLE (depth_projector defaults to it).
        det_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sub = self.create_subscription(
            Detection3DArray, det_topic, self._on_detections_3d, det_qos
        )
        self._pub_obj = self.create_publisher(
            SemanticEntityArray, obj_topic, 10
        )
        self._pub_mk = self.create_publisher(
            MarkerArray, mk_topic, 10
        )
        self._pub_dbg_mk = self.create_publisher(
            MarkerArray, debug_mk_topic, 10
        )
        self._pub_island_dbg_mk = self.create_publisher(
            MarkerArray, island_dbg_topic, 10
        )
        if (
            self._publish_split_visibility_markers
            and self._visible_split_mk_topic.strip()
            and self._remembered_split_mk_topic.strip()
        ):
            self._pub_mk_visible = self.create_publisher(
                MarkerArray,
                self._visible_split_mk_topic.strip(),
                10,
            )
            self._pub_mk_remembered = self.create_publisher(
                MarkerArray,
                self._remembered_split_mk_topic.strip(),
                10,
            )
        else:
            self._pub_mk_visible = None
            self._pub_mk_remembered = None
        # /map from slam_toolbox is published TRANSIENT_LOCAL (latched).
        # Use a matching QoS so we receive the very first map after a
        # late-startup, otherwise the aggregator can sit map-less even
        # though /map has been alive for ages.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._island = OccupancyIslandAssociator()
        self._sub_map = self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, map_qos
        )

        # Day 9 — TF + PointCloud2 cluster anchoring.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pc = PointCloudClusterAssociator(
            target_frame=self._frame_id,
            max_buffer_size=self._pc_buffer_size,
            max_points_per_cloud=self._pc_max_pts_per_cloud,
        )
        if self._use_pc_anchor and self._pc_topic:
            pc_qos = QoSProfile(
                depth=2,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )
            self._sub_pc = self.create_subscription(
                PointCloud2, self._pc_topic,
                self._on_pointcloud, pc_qos,
            )
        else:
            self._sub_pc = None
        # Day 9 anchor stats counters — incremented in the hot path,
        # serialised onto /semantic_map/anchor_debug_stats every
        # ``anchor_debug_stats_period_sec``.
        self._anchor_counters: Dict[str, int] = {
            "observations_total": 0,
            "pointcloud_anchor_success": 0,
            "occupancy_island_anchor_success": 0,
            "candidate_no_anchor": 0,
            "rejected_no_pointcloud": 0,
            "rejected_no_points_near_detection": 0,
            "rejected_no_occupied_island_nearby": 0,
            "rejected_wall_like_island": 0,
            "rejected_near_wall": 0,
            "rejected_pc_shape_invalid": 0,
            "rejected_pc_height_invalid": 0,
            "rejected_pc_too_small": 0,
            "rejected_pc_too_large": 0,
            "rejected_pc_tf_failed": 0,
            "promoted_confirmed_by_pc": 0,
            "promoted_confirmed_by_island": 0,
            "invalidated": 0,
            "pc_map_disagreement": 0,
        }
        # Per-class counters (sparse — created on first use). Encoded
        # as ``"<key>__<class>"`` so a single grep pulls every class
        # for a given counter on /semantic_map/anchor_debug_stats.
        self._anchor_counters_per_class: Dict[str, int] = {}
        if self._anchor_stats_topic and self._anchor_stats_period > 0.0:
            self._pub_anchor_stats = self.create_publisher(
                String, self._anchor_stats_topic, 10,
            )
            self._anchor_stats_timer = self.create_timer(
                self._anchor_stats_period, self._publish_anchor_stats,
            )
        else:
            self._pub_anchor_stats = None
            self._anchor_stats_timer = None
        # Diagnostics for /semantic_map/island_debug_markers — keep
        # only the last few rejection reasons + the snap-events from
        # the most recent detection callback so RViz isn't spammed.
        self._last_island_events: List[
            Tuple[str, float, float, IslandResult]
        ] = []
        ctrl_topic = str(self.get_parameter("control_topic").value)
        self._sub_ctrl = self.create_subscription(
            String, ctrl_topic, self._on_control, 10
        )
        self._timer = self.create_timer(hk_period, self._on_housekeep)

        self.get_logger().info(
            f"semantic_memory_aggregator ready. "
            f"in={det_topic} out={obj_topic} markers={mk_topic} "
            f"debug_markers={debug_mk_topic} "
            f"split_vis="
            f"{self._visible_split_mk_topic if self._pub_mk_visible else 'off'}/"
            f"{self._remembered_split_mk_topic if self._pub_mk_remembered else 'off'} "
            f"island_debug={island_dbg_topic} "
            f"map_in={map_topic} use_island={self._use_island} "
            f"nms_r={math.sqrt(self._nms_r2):.2f}m alpha={self._alpha} "
            f"decay_rate={self._conf_decay_rate}/s "
            f"min_det_conf={self._min_det_conf} "
            f"vis_timeout={self._vis_timeout}s "
            f"permanent_after={self._permanent_n} "
            f"confirmed_keep={self._keep_confirmed} "
            f"confirmed_min_conf={self._confirmed_min_conf} "
            f"merge_r={math.sqrt(self._merge_r2):.2f}m "
            f"merge_by_island={self._merge_by_island_id} "
            f"merge_class_radii={self._merge_radius_by_class} "
            f"min_obs_per_class={self._min_obs_to_confirm} "
            f"single_obs_island_allow={sorted(self._single_obs_island_allow)} "
            f"require_island_anchor={sorted(self._require_island_classes)} "
            f"max_confirmed_per_class={self._max_confirmed_per_class} "
            f"canonical_aliases={len(self._canonical_map)}"
        )

    # ------------------------------------------------------------------
    # OccupancyGrid callback — feeds the island associator
    # ------------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        # Cheap; just stash the latest grid. The hot path uses it
        # synchronously inside _on_detections_3d so a stale map is
        # better than no map.
        self._island.update_map(msg)

    # ------------------------------------------------------------------
    # PointCloud2 callback — feeds the cluster associator
    # ------------------------------------------------------------------
    def _on_pointcloud(self, msg: PointCloud2) -> None:
        # Single TF lookup + decimation per cloud. Keeps the per-detection
        # cost constant regardless of how many points the LiDAR pushes.
        ok = self._pc.update(msg, self._tf_buffer, logger=self.get_logger())
        if not ok:
            self._anchor_counters["rejected_pc_tf_failed"] += 1

    # ------------------------------------------------------------------
    # Helper — wrap the pointcloud cluster associator with class kwargs
    # ------------------------------------------------------------------
    def _maybe_associate_pointcloud(
        self,
        x: float,
        y: float,
        *,
        canonical_class: str = "",
    ) -> PointCloudResult:
        """Run the PointCloudClusterAssociator with class-specific kwargs."""
        if not self._use_pc_anchor:
            return PointCloudResult(
                success=False, snapped_x=x, snapped_y=y, snapped_z=0.0,
                cluster_id="", point_count=0, bbox_length_m=0.0,
                bbox_width_m=0.0, bbox_height_m=0.0,
                rejection_reason="pc_anchor_disabled",
            )
        if not self._pc.has_pointcloud():
            return PointCloudResult(
                success=False, snapped_x=x, snapped_y=y, snapped_z=0.0,
                cluster_id="", point_count=0, bbox_length_m=0.0,
                bbox_width_m=0.0, bbox_height_m=0.0,
                rejection_reason="no_pointcloud",
            )
        radius = self._pc_radius_by_class.get(
            canonical_class, self._pc_default_radius
        )
        # Per-class height slab; fallback to a permissive 0–3 m for
        # other classes (boxes/chairs).
        z_min, z_max = self._pc_z_by_class.get(
            canonical_class, (0.05, 3.0),
        )
        # Aspect-ratio limit. Tighter for person (people aren't long
        # thin objects); looser for tables (a long rectangular top
        # legitimately has aspect 3-4).
        aspect = 3.0 if canonical_class == "person" else 6.0
        return self._pc.associate(
            x, y,
            search_radius_m=radius,
            z_min=z_min, z_max=z_max,
            min_cluster_points=self._pc_min_pts,
            max_cluster_points=self._pc_max_pts,
            cluster_tolerance_m=self._pc_tol,
            max_aspect_ratio=aspect,
        )

    # ------------------------------------------------------------------
    # Anchor counter helpers
    # ------------------------------------------------------------------
    def _bump_anchor(self, key: str, cls: str = "") -> None:
        if key in self._anchor_counters:
            self._anchor_counters[key] += 1
        if cls:
            ck = f"{key}__{cls}"
            self._anchor_counters_per_class[ck] = (
                self._anchor_counters_per_class.get(ck, 0) + 1
            )

    def _publish_anchor_stats(self) -> None:
        """Day 9 Task 5 — single-line key=value snapshot.

        Every counter is included (even zero) so the diagnose script
        can grep deterministically. Per-class counters are appended
        only when non-zero to keep the line manageable.
        """
        if self._pub_anchor_stats is None:
            return
        # Snapshot of currently-tracked entities by anchor type. Cheap
        # to compute on each tick; gives the operator an at-a-glance
        # view of "how many real landmarks exist right now".
        n_pc_anchored = 0
        n_isl_anchored = 0
        n_no_anchor = 0
        n_invalid = 0
        for ent in self._entities.values():
            if ent.is_invalid:
                n_invalid += 1
                continue
            if ent.island_id.startswith("pc_"):
                n_pc_anchored += 1
            elif ent.island_id:
                n_isl_anchored += 1
            else:
                n_no_anchor += 1

        parts = [
            f"{k}={v}" for k, v in self._anchor_counters.items()
        ]
        parts.append(f"current_pc_anchored={n_pc_anchored}")
        parts.append(f"current_island_anchored={n_isl_anchored}")
        parts.append(f"current_unanchored={n_no_anchor}")
        parts.append(f"current_invalid={n_invalid}")
        parts.append(
            f"pointcloud_buffer={len(self._pc._frames) if hasattr(self._pc, '_frames') else 0}"
        )
        parts.append(
            f"map_available={int(self._island.has_map())}"
        )
        # Append per-class non-zero deltas so a single grep tells the
        # operator "person had pc_anchor_success_by_class=3 in the
        # last tick". Format: ``per_class_<key>__<class>=N``.
        for k, v in sorted(self._anchor_counters_per_class.items()):
            if v == 0:
                continue
            parts.append(f"per_class_{k}={v}")
        msg = String()
        msg.data = " ".join(parts)
        self._pub_anchor_stats.publish(msg)

    # ------------------------------------------------------------------
    # Canonical class map (Task 4)
    # ------------------------------------------------------------------
    def _canonicalize(self, raw_class: str) -> str:
        """Look up the canonical class name for a raw detector label.

        Falls through to the normalised raw label when no alias
        matches, so unknown classes (e.g. ``"box"``) still flow
        through unchanged.
        """
        key = _normalise_class_key(raw_class)
        return self._canonical_map.get(key, key)

    # ------------------------------------------------------------------
    # Operator hook — runtime control over the entity registry
    # ------------------------------------------------------------------
    def _on_control(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        # Tokenise so "clear_class chair" parses cleanly. Verbs are
        # case-insensitive; class arguments preserve the canonical
        # mapping (e.g. "Desk" → "table").
        parts = raw.split()
        verb = parts[0].lower()
        args = parts[1:]
        if verb in ("reset", "clear", "clear_all"):
            n = len(self._entities)
            self._entities.clear()
            self._next_id_per_class.clear()
            self.get_logger().warn(
                f"semantic_memory: control={verb!r} — wiped {n} entities "
                f"(including permanent landmarks). Re-publishing empty state."
            )
            self._publish_state(stamp=self._now_msg())
            return
        if verb in ("clear_non_permanent", "clear_transient"):
            kept: Dict[str, TrackedEntity] = {}
            dropped = 0
            for eid, ent in self._entities.items():
                is_permanent = (
                    self._permanent_n > 0
                    and ent.observations_count >= self._permanent_n
                )
                if is_permanent:
                    kept[eid] = ent
                else:
                    dropped += 1
            self._entities = kept
            self.get_logger().warn(
                f"semantic_memory: control={verb!r} — kept "
                f"{len(kept)} permanent, dropped {dropped} transient."
            )
            self._publish_state(stamp=self._now_msg())
            return
        if verb == "clear_invalid":
            kept_d: Dict[str, TrackedEntity] = {}
            dropped = 0
            for eid, ent in self._entities.items():
                if ent.is_invalid:
                    dropped += 1
                else:
                    kept_d[eid] = ent
            self._entities = kept_d
            self.get_logger().warn(
                f"semantic_memory: control={verb!r} — dropped "
                f"{dropped} invalid entities; {len(kept_d)} remain."
            )
            self._publish_state(stamp=self._now_msg())
            return
        if verb == "clear_class" and args:
            target_canon = self._canonicalize(args[0])
            kept_c: Dict[str, TrackedEntity] = {}
            dropped = 0
            for eid, ent in self._entities.items():
                if ent.class_label == target_canon:
                    dropped += 1
                else:
                    kept_c[eid] = ent
            self._entities = kept_c
            self.get_logger().warn(
                f"semantic_memory: control={verb!r} class={target_canon!r} "
                f"(req={args[0]!r}) — dropped {dropped} entities; "
                f"{len(kept_c)} remain."
            )
            self._publish_state(stamp=self._now_msg())
            return
        if verb == "clear_candidates":
            kept_cand: Dict[str, TrackedEntity] = {}
            dropped = 0
            for eid, ent in self._entities.items():
                if not ent.is_confirmed and not ent.is_invalid:
                    dropped += 1
                else:
                    kept_cand[eid] = ent
            self._entities = kept_cand
            self.get_logger().warn(
                f"semantic_memory: control={verb!r} — dropped "
                f"{dropped} candidate entities; {len(kept_cand)} remain."
            )
            self._publish_state(stamp=self._now_msg())
            return
        if verb == "clear_unanchored" and args:
            # Day 8++++ Task 3 — explicitly drop BOTH unanchored
            # candidates AND unanchored confirmed entities of the
            # target class (display_name "...|confirmed|-" and
            # "...|candidate|-" alike). Invalid entities are also
            # dropped — once an operator runs ``clear_unanchored
            # person`` the registry should not retain any person
            # without an island.
            target_canon = self._canonicalize(args[0])
            kept_u: Dict[str, TrackedEntity] = {}
            dropped_conf = 0
            dropped_cand = 0
            dropped_inv = 0
            for eid, ent in self._entities.items():
                if ent.class_label == target_canon and not ent.island_id:
                    if ent.is_invalid:
                        dropped_inv += 1
                    elif ent.is_confirmed:
                        dropped_conf += 1
                    else:
                        dropped_cand += 1
                    continue
                kept_u[eid] = ent
            self._entities = kept_u
            self.get_logger().warn(
                f"semantic_memory: control={verb!r} class={target_canon!r} "
                f"— dropped confirmed={dropped_conf} candidate={dropped_cand} "
                f"invalid={dropped_inv} unanchored {target_canon} entities; "
                f"{len(kept_u)} remain."
            )
            self._publish_state(stamp=self._now_msg())
            return
        if verb == "merge_class" and args:
            target_canon = self._canonicalize(args[0])
            before = len(self._entities)
            # Force a merge pass restricted to the target class by
            # temporarily growing the merge radius for that class to
            # something pragmatic (8 m). Tasks 3 + 7: an operator
            # asking for "merge_class person" wants ALL person
            # landmarks fused, not just the ones within the small
            # default radius.
            saved = self._merge_radius_by_class.get(target_canon)
            self._merge_radius_by_class[target_canon] = 8.0
            try:
                self._merge_close_entities()
            finally:
                if saved is None:
                    self._merge_radius_by_class.pop(target_canon, None)
                else:
                    self._merge_radius_by_class[target_canon] = saved
            after = len(self._entities)
            self.get_logger().warn(
                f"semantic_memory: control={verb!r} class={target_canon!r} "
                f"— merged {before - after} entities ({before} -> "
                f"{after} total)."
            )
            self._publish_state(stamp=self._now_msg())
            return
        if verb == "keep_best_class" and args:
            target_canon = self._canonicalize(args[0])
            class_ents = [
                (eid, ent) for eid, ent in self._entities.items()
                if ent.class_label == target_canon
            ]
            if not class_ents:
                self.get_logger().warn(
                    f"semantic_memory: control={verb!r} class={target_canon!r} "
                    f"— no entities match."
                )
                return
            class_ents.sort(
                key=lambda kv: self._quality_score(kv[1]), reverse=True
            )
            keeper_id = class_ents[0][0]
            dropped = 0
            kept_kb: Dict[str, TrackedEntity] = {}
            for eid, ent in self._entities.items():
                if ent.class_label != target_canon or eid == keeper_id:
                    kept_kb[eid] = ent
                else:
                    dropped += 1
            self._entities = kept_kb
            self.get_logger().warn(
                f"semantic_memory: control={verb!r} class={target_canon!r} "
                f"— kept {keeper_id} (quality="
                f"{self._quality_score(class_ents[0][1])}); "
                f"dropped {dropped} other {target_canon} entities."
            )
            self._publish_state(stamp=self._now_msg())
            return
        self.get_logger().warn(
            f"semantic_memory: unknown /semantic_map/control={raw!r}; "
            f"expected one of: reset / clear / clear_all / "
            f"clear_non_permanent / clear_transient / clear_invalid / "
            f"clear_candidates / 'clear_class <class>' / "
            f"'clear_unanchored <class>' / 'merge_class <class>' / "
            f"'keep_best_class <class>' (e.g. 'keep_best_class person')"
        )

    def _refresh_candidate_hint(
        self,
        ent: TrackedEntity,
        *,
        anchor_kind: str,
        island_res: IslandResult,
        pc_res: PointCloudResult,
        canonical_class: str,
    ) -> None:
        """Explain why an entity remains a candidate (RViz).

        For ``table``, occupancy island rejections never override a stable
        ``pc_*`` anchor — the debug tag reflects PC obs/conf gates instead.
        """
        if ent.is_invalid:
            ent.candidate_status_hint = ""
            return
        if ent.is_confirmed:
            ent.candidate_status_hint = ""
            return
        if canonical_class == "table":
            ent.candidate_status_hint = table_candidate_not_confirmed_tag(
                island_id=ent.island_id or "",
                observations_count=int(ent.observations_count),
                table_min_observations=int(
                    self._min_obs_to_confirm.get("table", 2)
                ),
                table_allow_single_pc_obs=bool(self._table_allow_single_pc),
                confidence=float(ent.confidence),
                confirmed_min_confidence=float(self._confirmed_min_conf),
                pc_cluster_success=bool(pc_res.success),
                island_rejection_reason=island_res.rejection_reason or "",
            )
            return
        # Non-table: same as legacy — occupancy rejection hints only when
        # truly unanchored.
        if anchor_kind != "none":
            ent.candidate_status_hint = ""
            return
        if (ent.island_id or "").strip():
            ent.candidate_status_hint = ""
            return
        ent.candidate_status_hint = candidate_not_confirmed_hint_from_rejection(
            island_res.rejection_reason
        )

    # ------------------------------------------------------------------
    # Detection3DArray callback — the hot path
    # ------------------------------------------------------------------
    def _on_detections_3d(self, msg: Detection3DArray) -> None:
        self._n_messages += 1
        now = self.get_clock().now()
        # Reset the island-debug event buffer for this frame so the
        # marker overlay only ever shows reasons from the most recent
        # detection callback (otherwise rejected reasons pile up
        # forever).
        self._last_island_events = []

        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            raw_class = str(hyp.class_id)
            score = float(hyp.score)
            canonical = self._canonicalize(raw_class)
            eff_floor = effective_detection_confidence_floor(
                self._min_det_conf,
                canonical,
                self._cand_min_person,
                self._cand_min_table,
            )
            if score < eff_floor:
                continue
            x = float(det.bbox.center.position.x)
            y = float(det.bbox.center.position.y)
            z = float(det.bbox.center.position.z)

            # 1) Canonicalise the class label so "desk" / "workbench"
            #    collapse onto the canonical "table" entity. The raw
            #    label is preserved on the entity for debugging.
            self._anchor_counters["observations_total"] += 1
            self._bump_anchor("observations_total", canonical)

            # 2) Day 9 — anchor priority order:
            #      A. PointCloud2 cluster anchor (primary)
            #      B. Occupancy-island anchor (fallback / cross-validation)
            #      C. Unanchored candidate (no selection eligibility)
            #
            #    Run BOTH so the cross-validation logic in step 3 can
            #    spot pc/map disagreements. The pc anchor wins by
            #    default; if pc fails but map succeeds, the entity
            #    falls back to an isl_ anchor.
            pc_res = self._maybe_associate_pointcloud(
                x, y, canonical_class=canonical,
            )
            island_res = self._maybe_associate_island(
                x, y, canonical_class=canonical
            )

            # Record event tuple for the debug overlay (we pass both
            # results through so the marker layer can colour-code pc
            # success vs island success).
            self._last_island_events.append(
                (raw_class, x, y, island_res)
            )

            # Choose the anchor according to priority, and record stats.
            if pc_res.success:
                snap_x = pc_res.snapped_x
                snap_y = pc_res.snapped_y
                anchor_id = pc_res.cluster_id  # pc_*
                anchor_kind = "pc"
                self._bump_anchor(
                    "pointcloud_anchor_success", canonical,
                )
                # Cross-validation (Day 9 Task 3): if pc found a
                # cluster but island says wall_like / shape_invalid,
                # we trust pc but log the disagreement so the operator
                # can spot wall fragments masquerading as people.
                if (not island_res.success
                        and island_res.rejection_reason
                        in self._invalidating_reasons):
                    self._anchor_counters[
                        "pc_map_disagreement"
                    ] += 1
                    self.get_logger().info(
                        f"pc_map_disagreement {canonical!r} pc={pc_res.cluster_id} "
                        f"map_reason={island_res.rejection_reason!r} — "
                        f"pc anchor wins."
                    )
            elif island_res.success:
                snap_x = island_res.snapped_x
                snap_y = island_res.snapped_y
                anchor_id = island_res.island_id  # isl_*
                anchor_kind = "island"
                self._bump_anchor(
                    "occupancy_island_anchor_success", canonical,
                )
            else:
                snap_x, snap_y = x, y
                anchor_id = ""
                anchor_kind = "none"
                self._bump_anchor("candidate_no_anchor", canonical)
                # Map per-rejection-reason buckets onto the named
                # counters so the diagnose script can identify which
                # of (pc, map) was the bottleneck.
                pcr = pc_res.rejection_reason
                if pcr == "no_pointcloud":
                    self._anchor_counters["rejected_no_pointcloud"] += 1
                elif pcr == "no_points_near_detection":
                    self._anchor_counters[
                        "rejected_no_points_near_detection"
                    ] += 1
                elif pcr == "cluster_too_small":
                    self._anchor_counters["rejected_pc_too_small"] += 1
                elif pcr == "cluster_too_large":
                    self._anchor_counters["rejected_pc_too_large"] += 1
                elif pcr == "cluster_height_invalid":
                    self._anchor_counters["rejected_pc_height_invalid"] += 1
                elif pcr == "cluster_shape_invalid":
                    self._anchor_counters["rejected_pc_shape_invalid"] += 1
                isr = island_res.rejection_reason
                if isr == "no_occupied_island_nearby":
                    self._anchor_counters[
                        "rejected_no_occupied_island_nearby"
                    ] += 1
                elif isr == "wall_like_island":
                    self._anchor_counters["rejected_wall_like_island"] += 1
                elif isr in (
                    "person_too_close_to_wall",
                    "table_too_close_to_wall",
                    "too_close_to_wall",
                ):
                    self._anchor_counters["rejected_near_wall"] += 1

            # 3) Strict-anchor short-circuit. With pc anchoring on,
            #    the operator can choose to keep candidates around for
            #    debugging (default) OR drop the detection entirely.
            #    We only short-circuit when BOTH pc and island failed
            #    AND keep_unanchored is False — the user's contract
            #    says map-unavailable should NOT block.
            both_failed = not pc_res.success and not island_res.success
            map_unavailable = island_res.rejection_reason == (
                "occupied_map_unavailable"
            )
            if (
                both_failed
                and not self._keep_unanchored
                and not map_unavailable
            ):
                continue
            island_id = anchor_id  # legacy local name kept for the
            # rest of the function.

            # 4) NMS / data association on canonical class +
            #    island-snapped position.
            matched = self._match_existing(canonical, snap_x, snap_y, z)
            if matched is not None:
                # Update in place. We EMA-blend the position toward
                # the (snapped) sample. Snapping makes the EMA
                # converge on the island centroid almost immediately,
                # which is exactly the "stop dancing around" property
                # we want.
                a = self._alpha
                matched.px = (1.0 - a) * matched.px + a * snap_x
                matched.py = (1.0 - a) * matched.py + a * snap_y
                matched.pz = (1.0 - a) * matched.pz + a * z
                matched.confidence = min(
                    1.0, matched.confidence + self._conf_up
                )
                matched.observations_count += 1
                matched.last_seen = now
                matched.currently_visible = True
                # Track same-island re-observation count for the
                # third promotion path. Reset if the entity drifts
                # to a different island.
                if island_id:
                    if matched.island_id == island_id:
                        matched.same_island_observations += 1
                    else:
                        # Snap the entity onto the new island and
                        # restart the same-island counter. Keeps the
                        # entity from clinging to a stale anchor when
                        # /map updates change the cluster geometry.
                        matched.island_id = island_id
                        matched.same_island_observations = 1
                if not matched.raw_class and raw_class:
                    matched.raw_class = raw_class
                self._n_associations += 1
                # Task 3 — accumulate "tried to anchor on a wall
                # again" evidence on confirmed entities. Invalid
                # signal repeatedly hitting the same confirmed
                # landmark eventually retires it as a false positive.
                # Day 9: only count the wall hit if the PC anchor ALSO
                # failed. A successful pc anchor with a failed map
                # anchor is a pc_map_disagreement, NOT a wall-hit.
                if (
                    self._allow_invalid
                    and matched.is_confirmed
                    and not island_res.success
                    and not pc_res.success
                    and island_res.rejection_reason
                    in self._invalidating_reasons
                ):
                    matched.invalid_evidence_count += 1
                    if (
                        not matched.is_invalid
                        and matched.invalid_evidence_count
                        >= max(1, self._invalid_thresh)
                    ):
                        matched.is_invalid = True
                        self._anchor_counters["invalidated"] += 1
                        self._bump_anchor("invalidated", matched.class_label)
                        self.get_logger().warn(
                            f"INVALIDATED confirmed entity "
                            f"{matched.entity_id} cls="
                            f"{matched.class_label!r} after "
                            f"{matched.invalid_evidence_count} bad "
                            f"island events (last="
                            f"{island_res.rejection_reason!r}). "
                            f"Marker rerouted to /semantic_map/"
                            f"debug_markers; target_selector will "
                            f"skip it."
                        )
                elif (
                    self._allow_invalid
                    and matched.is_invalid
                    and (island_res.success or pc_res.success)
                ):
                    # Recovery: a fresh successful island association
                    # outweighs accumulated bad evidence (slowly). We
                    # don't auto-clear is_invalid here — the operator
                    # can /semantic_map/control "clear_invalid" when
                    # they're confident — but we DO decay the counter
                    # so a stable re-observation halts further damage.
                    matched.invalid_evidence_count = max(
                        0, matched.invalid_evidence_count - 1
                    )
                self._refresh_candidate_hint(
                    matched,
                    anchor_kind=anchor_kind,
                    island_res=island_res,
                    pc_res=pc_res,
                    canonical_class=canonical,
                )
                self._maybe_promote_to_confirmed(matched)
            else:
                eid = self._mint_id(canonical)
                ent = TrackedEntity(
                    entity_id=eid,
                    class_label=canonical,
                    px=snap_x, py=snap_y, pz=z,
                    confidence=min(1.0, max(0.0, score)),
                    observations_count=1,
                    first_seen=now,
                    last_seen=now,
                    currently_visible=True,
                    raw_class=raw_class,
                    island_id=island_id,
                    is_confirmed=False,
                    same_island_observations=1 if island_id else 0,
                )
                self._entities[eid] = ent
                self._n_new_entities += 1
                self._refresh_candidate_hint(
                    ent,
                    anchor_kind=anchor_kind,
                    island_res=island_res,
                    pc_res=pc_res,
                    canonical_class=canonical,
                )
                self._maybe_promote_to_confirmed(ent)
                anchor_str = (
                    f" island={island_id}" if island_id
                    else f" UNANCHORED({island_res.rejection_reason})"
                )
                self.get_logger().info(
                    f"new entity {eid} canonical={canonical!r} "
                    f"raw={raw_class!r} pos=({snap_x:.2f},{snap_y:.2f},"
                    f"{z:.2f}) conf={score:.2f}{anchor_str}"
                )

        # Publish persistent state + the freshly populated island
        # debug overlay every input message so consumers don't have
        # to wait for the 1 Hz housekeeping tick.
        self._publish_state(stamp=msg.header.stamp)
        self._publish_island_debug(stamp=msg.header.stamp)
        self._tick_log()

    # ------------------------------------------------------------------
    # Helper — wrap the associator with the node's parameter set.
    # ------------------------------------------------------------------
    def _maybe_associate_island(
        self,
        x: float,
        y: float,
        *,
        canonical_class: str = "",
        collect_cells: bool = False,
    ) -> IslandResult:
        """Run the OccupancyIslandAssociator for ``canonical_class``.

        Class-specific island shape + wall-clearance overrides are
        looked up in ``self._class_constraints``. Unknown classes
        (e.g. ``"box"``) fall through with all overrides set to
        their disabled / sentinel value, so they retain the legacy
        global behaviour.
        """
        if not self._use_island:
            return IslandResult(
                False, x, y, "", 0, 0.0, 0.0,
                "occupied_map_unavailable",
            )
        c = self._class_constraints.get(canonical_class) or {}
        return self._island.associate(
            x, y,
            search_radius_m=self._island_radius,
            occupied_threshold=self._occ_thresh,
            min_island_cells=self._island_min,
            max_island_cells=self._island_max,
            reject_wall_like=self._reject_walls,
            wall_aspect_ratio=self._wall_aspect,
            wall_min_length_m=self._wall_min_len,
            class_name=canonical_class,
            class_min_island_cells=c.get("min_cells"),
            class_max_island_cells=c.get("max_cells"),
            class_max_island_length_m=c.get("max_length_m"),
            class_max_island_width_m=c.get("max_width_m"),
            class_max_island_aspect=c.get("max_aspect"),
            class_min_wall_clearance_m=float(
                c.get("min_wall_clearance_m", 0.0) or 0.0
            ),
            wall_clearance_min_external_cells=self._wall_clear_min_ext,
            wall_clearance_min_long_side_m=self._wall_clear_min_long,
            collect_cells=collect_cells,
        )

    # ------------------------------------------------------------------
    # Promotion: candidate -> confirmed (Task 2)
    # ------------------------------------------------------------------
    def _maybe_promote_to_confirmed(self, ent: TrackedEntity) -> None:
        """Apply the three promotion paths from the spec, gated by
        per-class minimums (Task 4).

        Promote ``ent`` to confirmed if ANY of:
          (table) Path A — ``table_pc_anchor`` when ``pc_*`` anchor is
              stable (occupancy island optional);
          (a) observations_count >= permanent_after_observations
              AND observations_count >= class min_observations
          (b) island_id is non-empty AND
              confidence >= island_promotion_confidence AND
              EITHER observations_count >= class_min_observations
              OR class is on the single-obs island allow-list
          (c) island_id is non-empty AND
              same_island_observations >= island_promotion_count AND
              same_island_observations >= class_min_observations

        Does nothing if the entity is already confirmed.

        Notes
        -----
        Canonical ``table`` first checks the LiDAR ``pc_*`` fast path
        (Path A): occupancy island association is *not* required when
        the cluster centroid is stable. Path B/C reuse the legacy obs
        and island-recurrence gates for ``isl_*`` or high observation
        counts.
        """
        if ent.is_confirmed:
            return
        # Invalidated entities cannot be re-promoted while still
        # tagged invalid; the operator must /semantic_map/control
        # clear_invalid (or clear_class) first.
        if ent.is_invalid:
            return

        cls = ent.class_label
        class_min_obs = max(1, int(self._min_obs_to_confirm.get(cls, 1)))
        same_island_or_obs = max(
            ent.observations_count, ent.same_island_observations
        )
        single_obs_allowed = cls in self._single_obs_island_allow

        path = ""
        if cls == "table" and table_promote_via_pc_anchor_path(
            observations_count=ent.observations_count,
            table_min_observations=class_min_obs,
            table_allow_single_pc_obs=bool(self._table_allow_single_pc),
            confidence=float(ent.confidence),
            confirmed_min_confidence=float(self._confirmed_min_conf),
            island_id=ent.island_id or "",
            is_invalid=False,
        ):
            path = "table_pc_anchor"
        if not path:
            if (
                self._permanent_n > 0
                and ent.observations_count >= self._permanent_n
                and ent.observations_count >= class_min_obs
            ):
                path = "obs_count"
            elif (
                ent.island_id
                and self._island_promo_conf <= 1.0
                and ent.confidence >= self._island_promo_conf
                and (single_obs_allowed or same_island_or_obs >= class_min_obs)
            ):
                path = "island_anchor+conf"
            elif (
                ent.island_id
                and self._island_promo_count > 0
                and ent.same_island_observations >= self._island_promo_count
                and same_island_or_obs >= class_min_obs
            ):
                path = "island_recurrence"
        if not path:
            return
        if float(ent.confidence) + 1e-9 < float(self._confirmed_min_conf):
            return
        if promotion_blocked_without_anchor(
            path,
            cls,
            ent.island_id,
            self._require_island_classes,
        ):
            return
        hint = (ent.candidate_status_hint or "").strip()
        table_pc = cls == "table" and (ent.island_id or "").startswith(
            "pc_",
        )
        if not table_pc:
            if hint in frozenset({"wall_like_island", "near_unknown"}):
                return
        ent.is_confirmed = True
        # Clamp confidence at the floor immediately so the very next
        # housekeeping decay doesn't push a fresh-confirmed entity
        # below the floor before its first re-publish.
        if ent.confidence < self._confirmed_min_conf:
            ent.confidence = self._confirmed_min_conf
        # Day 9 — break promotion stats out by anchor type so the
        # operator can see "promoted_by_pc" vs "promoted_by_island"
        # on /semantic_map/anchor_debug_stats.
        if ent.island_id.startswith("pc_"):
            self._anchor_counters["promoted_confirmed_by_pc"] += 1
            self._bump_anchor("promoted_confirmed_by_pc", cls)
        elif ent.island_id.startswith("isl_"):
            self._anchor_counters["promoted_confirmed_by_island"] += 1
            self._bump_anchor("promoted_confirmed_by_island", cls)
        self.get_logger().info(
            f"PROMOTED to confirmed: {ent.entity_id} "
            f"cls={ent.class_label!r} via={path} "
            f"n={ent.observations_count} conf={ent.confidence:.2f} "
            f"anchor={ent.island_id or '-'}"
        )

    # ------------------------------------------------------------------
    # Housekeeping timer — slow path (1 Hz)
    # ------------------------------------------------------------------
    def _on_housekeep(self) -> None:
        """Decay confidences (age-aware), mark stale invisible, prune,
        then merge same-class duplicates that drifted apart.

        Day 8+ behaviour:
          * Confirmed (= ``is_confirmed``) landmarks are persistent:
            their confidence is clamped at ``confirmed_min_confidence``
            and they are never pruned. They still get
            ``currently_visible=False`` when stale, but the marker
            stays on the map.
          * Candidate landmarks decay exponentially as before and get
            pruned once they fall below the conf+age thresholds.
        """
        now = self.get_clock().now()
        vis_ns = int(self._vis_timeout * 1e9)
        prune_ns = int(self._prune_age * 1e9)

        to_drop: List[str] = []
        for eid, ent in self._entities.items():
            age_ns = (now - ent.last_seen).nanoseconds
            age_s = age_ns / 1e9
            # Re-evaluate confirmed status; gives the obs-count
            # promotion path a chance to fire on the slow tick when a
            # detection-callback promotion was missed (e.g. exactly
            # at the threshold).
            self._maybe_promote_to_confirmed(ent)

            if not ent.is_confirmed:
                # Age-aware exponential decay (only candidates decay).
                ent.confidence *= math.exp(-self._conf_decay_rate * age_s)
            elif self._keep_confirmed:
                # Confirmed landmarks have a confidence floor so they
                # never accidentally fall below pruning thresholds and
                # stay solidly rendered in RViz across a long sweep.
                if ent.confidence < self._confirmed_min_conf:
                    ent.confidence = self._confirmed_min_conf
            if age_ns > vis_ns:
                ent.currently_visible = False
            if (
                not ent.is_confirmed
                and ent.confidence < self._prune_conf
                and age_ns > prune_ns
            ):
                to_drop.append(eid)
        for eid in to_drop:
            self.get_logger().info(
                f"pruning candidate {eid} (conf<{self._prune_conf}, "
                f"age>{self._prune_age:.0f}s)"
            )
            del self._entities[eid]

        # Second-pass same-class + same-island merge.
        self._merge_close_entities()

        # Day 8++ — apply per-class confirmed caps. Demoting (rather
        # than deleting) means an over-cap entity can get cleaned by
        # the candidate decay path naturally, and the operator can
        # see what was demoted in the debug stream until then.
        self._enforce_max_confirmed_caps()

        # Day 8++++ Task 3 — retroactively enforce island-anchor
        # requirement on confirmed entities of classes named in
        # ``require_island_anchor_for_classes``. Older runs (or
        # operator-set parameters that loosened the rule for a few
        # frames) may have left a ``person|confirmed|-`` entity in
        # the registry; we either demote it, or — if the operator
        # has opted in — flag it as ``is_invalid`` so it gets the
        # red INVALID marker treatment in RViz.
        self._enforce_island_anchor_required()

        # Re-publish so /semantic_map/objects has fresh confidence
        # values even when no detections arrived.
        stamp = self._now_msg()
        self._publish_state(stamp=stamp)
        self._publish_island_debug(stamp=stamp)

    def _class_merge_radius_m(self, canonical_class: str) -> float:
        """Pick the merge radius for a canonical class.

        Looks up class-specific overrides (person / table / chair) and
        falls back to ``entity_merge_radius_m`` for everything else.
        Returns 0 when merging is globally disabled (legacy default
        was 0 meaning off).
        """
        r = self._merge_radius_by_class.get(canonical_class)
        if r is None:
            return math.sqrt(self._merge_r2) if self._merge_r2 > 0.0 else 0.0
        return float(r) if r > 0.0 else 0.0

    def _merge_close_entities(self) -> None:
        """Fuse pairs of same-canonical-class entities.

        Two pairing rules trigger a merge:
          * Geometric: pair within the class-specific merge radius.
          * Island-id: pair sharing a non-empty ``island_id``
            (respected only when ``merge_by_island_id`` is True).

        The surviving entity is the higher-quality one
        (confirmed wins → more observations wins → older first_seen
        wins). Position is observation-count-weighted; confidence
        is max; observations_count is summed; same_island_observations
        is summed; raw_class is kept from the survivor unless empty.
        """
        items = list(self._entities.items())
        if not items:
            return
        merged_ids: set[str] = set()
        for i in range(len(items)):
            eid_i, ei = items[i]
            if eid_i in merged_ids:
                continue
            for j in range(i + 1, len(items)):
                eid_j, ej = items[j]
                if eid_j in merged_ids:
                    continue
                if ei.class_label != ej.class_label:
                    continue
                # Pairing rule 1: shared island_id (strongest signal).
                same_island = (
                    self._merge_by_island_id
                    and bool(ei.island_id)
                    and ei.island_id == ej.island_id
                )
                # Pairing rule 2: within geometric radius.
                radius_m = self._class_merge_radius_m(ei.class_label)
                radius2 = radius_m * radius_m if radius_m > 0.0 else 0.0
                dx = ei.px - ej.px
                dy = ei.py - ej.py
                dz = ei.pz - ej.pz
                d2 = dx * dx + dy * dy + dz * dz
                close_enough = (radius2 > 0.0 and d2 < radius2)
                if not (same_island or close_enough):
                    continue

                # Survivor ordering matches ``_quality_score`` (below).
                qi, qj = self._quality_score(ei), self._quality_score(ej)
                if qi > qj:
                    keep, drop, drop_id = ei, ej, eid_j
                elif qj > qi:
                    keep, drop, drop_id = ej, ei, eid_i
                elif eid_i <= eid_j:
                    keep, drop, drop_id = ei, ej, eid_j
                else:
                    keep, drop, drop_id = ej, ei, eid_i

                cand_into_confirmed = (
                    keep.is_confirmed != drop.is_confirmed
                    and (keep.is_confirmed or drop.is_confirmed)
                )

                wk = float(max(1, keep.observations_count))
                wd = float(max(1, drop.observations_count))
                tot = wk + wd
                keep.px = (wk * keep.px + wd * drop.px) / tot
                keep.py = (wk * keep.py + wd * drop.py) / tot
                keep.pz = (wk * keep.pz + wd * drop.pz) / tot
                keep.observations_count += drop.observations_count
                keep.same_island_observations += drop.same_island_observations
                keep.confidence = max(keep.confidence, drop.confidence)
                keep.is_confirmed = keep.is_confirmed or drop.is_confirmed
                if not keep.island_id and drop.island_id:
                    keep.island_id = drop.island_id
                if not keep.raw_class and drop.raw_class:
                    keep.raw_class = drop.raw_class
                if drop.last_seen.nanoseconds > keep.last_seen.nanoseconds:
                    keep.last_seen = drop.last_seen
                    keep.currently_visible = (
                        keep.currently_visible or drop.currently_visible
                    )
                if drop.first_seen.nanoseconds < keep.first_seen.nanoseconds:
                    keep.first_seen = drop.first_seen
                if keep.is_confirmed or (keep.island_id or "").strip():
                    keep.candidate_status_hint = ""
                elif not keep.candidate_status_hint:
                    keep.candidate_status_hint = drop.candidate_status_hint

                reason_parts = [
                    (
                        f"duplicate_merged_same_anchor={ei.island_id}"
                        if same_island else
                        f"duplicate_merged_nearby_same_class "
                        f"d={math.sqrt(d2):.2f}m r={radius_m:.2f}m"
                    )
                ]
                if cand_into_confirmed:
                    reason_parts.append(
                        "duplicate_candidate_merged_into_confirmed"
                    )
                merge_reason = "; ".join(reason_parts)
                self.get_logger().info(
                    f"merged duplicate entity {drop_id} -> "
                    f"{keep.entity_id} cls={keep.class_label!r} "
                    f"reason={merge_reason} "
                    f"n_total={keep.observations_count} "
                    f"confirmed={keep.is_confirmed}"
                )
                merged_ids.add(drop_id)
                if drop_id == eid_i:
                    break
        for eid in merged_ids:
            self._entities.pop(eid, None)

    # ------------------------------------------------------------------
    # Per-class confirmed caps (Task 3) + quality score helper
    # ------------------------------------------------------------------
    @staticmethod
    def _quality_score(ent: "TrackedEntity") -> tuple:
        """Per-entity quality vector for sorting ("best -> worst").

        Implementation lives in ``merge_quality_tuple`` for smoke-testing.
        """
        return merge_quality_tuple(
            is_invalid=bool(ent.is_invalid),
            is_confirmed=bool(ent.is_confirmed),
            island_id=ent.island_id or "",
            invalid_evidence_count=int(ent.invalid_evidence_count),
            observations_count=int(ent.observations_count),
            confidence=float(ent.confidence),
            currently_visible=bool(ent.currently_visible),
            first_seen_ns=int(ent.first_seen.nanoseconds),
        )

    def _enforce_max_confirmed_caps(self) -> None:
        """Cap the number of confirmed entities per class.

        Entities beyond the cap are *demoted* (``is_confirmed=False``)
        rather than deleted: the user gets to see them in the debug
        marker stream, and the candidate decay/prune path will retire
        them naturally if no fresh evidence arrives. We never demote
        entities tagged ``is_invalid`` because they're already
        non-selectable; they decay through the prune path.
        """
        for cls, cap in self._max_confirmed_per_class.items():
            if cap <= 0:
                continue
            confirmed = [
                ent for ent in self._entities.values()
                if ent.class_label == cls
                and ent.is_confirmed
                and not ent.is_invalid
            ]
            if len(confirmed) <= cap:
                continue
            # Sort best -> worst and demote the tail.
            confirmed.sort(key=self._quality_score, reverse=True)
            for tail in confirmed[cap:]:
                tail.is_confirmed = False
                self.get_logger().warn(
                    f"DEMOTED over-cap confirmed entity {tail.entity_id} "
                    f"cls={cls!r} (cap={cap}, total={len(confirmed)}). "
                    f"duplicate_suppressed_by_keep_best "
                    f"Marker rerouted to /semantic_map/debug_markers."
                )

    def _enforce_island_anchor_required(self) -> None:
        """Day 8++++ Task 3 — retroactive island-anchor enforcement.

        The user has classes (default: ``person``) where a confirmed
        entity *must* be anchored to an obstacle island. Older runs,
        loosened parameters, or single-frame promotion bugs can leave
        a stale ``person|confirmed|-`` entity in memory. This sweep
        finds every such entity and either demotes or invalidates it
        based on ``mark_unanchored_required_classes_invalid``. Both
        outcomes:
          * remove it from /semantic_map/markers (publication gate
            already filters confirmed-without-island for these classes)
          * remove it from target_selector candidate pool
          * surface it on /semantic_map/debug_markers so the operator
            can spot the offender visually.
        """
        if not self._require_island_classes:
            return
        downgraded = 0
        invalidated = 0
        for ent in self._entities.values():
            if ent.is_invalid:
                continue
            if ent.class_label not in self._require_island_classes:
                continue
            if ent.island_id:
                continue
            if not ent.is_confirmed:
                # Candidate without an island is fine; it stays a
                # candidate until promoted (which will require an
                # island anyway, see _maybe_promote_to_confirmed).
                continue
            if self._mark_unanchored_invalid:
                ent.is_invalid = True
                invalidated += 1
            else:
                ent.is_confirmed = False
                downgraded += 1
            self.get_logger().warn(
                f"RETRO-ENFORCE island anchor: {ent.entity_id} "
                f"class={ent.class_label!r} obs={ent.observations_count} "
                f"-> {'INVALID' if self._mark_unanchored_invalid else 'demoted to candidate'}. "
                f"Reason: confirmed but no island_id under "
                f"require_island_anchor_for_classes."
            )
        if downgraded or invalidated:
            self.get_logger().info(
                f"island-anchor retro sweep: downgraded={downgraded} "
                f"invalidated={invalidated} "
                f"classes={sorted(self._require_island_classes)}"
            )

    @staticmethod
    def _split_visibility_marker_pair_ids(entity_id: str) -> Tuple[int, int]:
        """Stable Marker.id pair for cyl / text under split RViz topics."""
        h = hash(entity_id) & 0x1FFFFFFF
        base = max(h << 2, 0)
        return base, base + 1

    def _make_confirmed_landmark_marker_pair(
        self,
        ent: TrackedEntity,
        stamp: TimeMsg,
        *,
        cyl_id: int,
        txt_id: int,
        entities_ns: str,
        labels_ns: str,
        r: float,
        g: float,
        b: float,
        alpha: float,
        label_text: str,
        text_color: ColorRGBA,
    ) -> Tuple[Marker, Marker]:
        cyl = Marker()
        cyl.header.stamp = stamp
        cyl.header.frame_id = self._frame_id
        cyl.ns = entities_ns
        cyl.id = cyl_id
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD
        cyl.pose.position.x = float(ent.px)
        cyl.pose.position.y = float(ent.py)
        cyl.pose.position.z = 0.20
        cyl.pose.orientation.w = 1.0
        cyl.scale.x = 0.30
        cyl.scale.y = 0.30
        cyl.scale.z = 0.40
        cyl.color = ColorRGBA(r=r, g=g, b=b, a=alpha)

        txt = Marker()
        txt.header.stamp = stamp
        txt.header.frame_id = self._frame_id
        txt.ns = labels_ns
        txt.id = txt_id
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = float(ent.px)
        txt.pose.position.y = float(ent.py)
        txt.pose.position.z = float(ent.pz) + 0.40
        if txt.pose.position.z < 0.55:
            txt.pose.position.z = 0.55
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.30
        txt.color = text_color
        txt.text = label_text
        return cyl, txt

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _publish_state(self, stamp: TimeMsg) -> None:
        # SemanticEntityArray
        arr = SemanticEntityArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self._frame_id
        for ent in self._entities.values():
            e = SemanticEntity()
            e.header.stamp = stamp
            e.header.frame_id = self._frame_id
            e.entity_id = ent.entity_id
            # class_label is the CANONICAL class so target_selector
            # can match "table" against an entity even when YOLOE
            # originally said "desk".
            e.class_label = ent.class_label
            # display_name carries the raw detector label and a
            # confirmed/candidate/invalid marker so RViz / debug
            # consumers can see provenance without a separate field.
            # Format:
            #   "<raw_class>|<confirmed|candidate|invalid>|<island_id_or_->"
            # target_selector reads "|invalid|" to skip the entity.
            if ent.is_invalid:
                status = "invalid"
            elif ent.is_confirmed:
                status = "confirmed"
            else:
                status = "candidate"
            display_island = ent.island_id or "-"
            display_raw = ent.raw_class or ent.class_label
            e.display_name = f"{display_raw}|{status}|{display_island}"
            e.pose_map.position.x = float(ent.px)
            e.pose_map.position.y = float(ent.py)
            e.pose_map.position.z = float(ent.pz)
            e.pose_map.orientation.w = 1.0
            e.size_xyz = Vector3(x=0.0, y=0.0, z=0.0)
            e.confidence = float(max(0.0, min(1.0, ent.confidence)))
            e.observations_count = int(ent.observations_count)
            e.first_seen = ent.first_seen.to_msg()
            e.last_seen = ent.last_seen.to_msg()
            e.currently_visible = bool(ent.currently_visible)
            e.is_dynamic = False
            e.uncertainty = float(1.0 - e.confidence)
            arr.entities.append(e)
        self._pub_obj.publish(arr)

        # MarkerArray — split the entity stream into:
        #   /semantic_map/markers       confirmed only (stable, persistent)
        #   /semantic_map/debug_markers candidates / invalid-like (noisy)
        # Optional per-visibility splits for RViz (demo recording):
        #   /semantic_map/markers_visible   — confirmed ∩ currently_visible
        #   /semantic_map/markers_remembered — confirmed ∩ ¬currently_visible
        confirmed_mk = MarkerArray()
        candidate_mk = MarkerArray()
        for arr_, ns_prefix in (
            (confirmed_mk, "confirmed"),
            (candidate_mk, "candidate"),
        ):
            clear = Marker()
            clear.action = Marker.DELETEALL
            clear.header.stamp = stamp
            clear.header.frame_id = self._frame_id
            clear.ns = ns_prefix
            arr_.markers.append(clear)

        visible_mk = MarkerArray()
        remembered_mk = MarkerArray()
        if self._pub_mk_visible is not None:
            for sns in (
                "visible_landmark_entities",
                "visible_landmark_labels",
            ):
                clr = Marker()
                clr.action = Marker.DELETEALL
                clr.header.stamp = stamp
                clr.header.frame_id = self._frame_id
                clr.ns = sns
                visible_mk.markers.append(clr)
            for sns in (
                "remembered_landmark_entities",
                "remembered_landmark_labels",
            ):
                clr = Marker()
                clr.action = Marker.DELETEALL
                clr.header.stamp = stamp
                clr.header.frame_id = self._frame_id
                clr.ns = sns
                remembered_mk.markers.append(clr)

        for i, ent in enumerate(self._entities.values()):
            r, g, b = _class_to_color(ent.class_label)
            # Day 9 Task 6 — short, class-coloured, large text labels.
            text_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            short_class = ent.class_label
            skip_plain_cylinder = False
            if ent.is_invalid:
                r = max(r, 0.85)
                g = min(g, 0.25)
                b = min(b, 0.25)
                alpha = 0.45
                ns_prefix = "invalid"
                target_arr = candidate_mk
                label_text = "REJECT invalid"
                text_color = ColorRGBA(r=1.0, g=0.45, b=0.10, a=1.0)
            elif ent.is_confirmed:
                requires_anchor = (
                    ent.class_label in self._require_island_classes
                )
                anchor_missing = requires_anchor and not ent.island_id
                if anchor_missing:
                    alpha = 0.45
                    ns_prefix = "confirmed_no_island"
                    target_arr = candidate_mk
                    label_text = (
                        f"REJECT {short_class} no_pc/no_island"
                    )
                    text_color = ColorRGBA(r=1.0, g=0.55, b=0.20, a=1.0)
                else:
                    alpha = 0.95 if ent.currently_visible else 0.55
                    state_word = (
                        "visible" if ent.currently_visible else "remembered"
                    )
                    label_text = (
                        f"{short_class} conf={ent.confidence:.2f} "
                        f"n={ent.observations_count} {state_word}"
                    )
                    if ent.class_label == "person":
                        text_color = ColorRGBA(
                            r=0.20, g=1.00, b=1.00, a=1.0,
                        )
                    elif ent.class_label == "table":
                        text_color = ColorRGBA(
                            r=1.00, g=0.95, b=0.20, a=1.0,
                        )
                    else:
                        text_color = ColorRGBA(
                            r=0.30, g=1.00, b=0.30, a=1.0,
                        )
                    leg_cyl, leg_txt = (
                        self._make_confirmed_landmark_marker_pair(
                            ent,
                            stamp,
                            cyl_id=i * 2 + 0,
                            txt_id=i * 2 + 1,
                            entities_ns="confirmed_entities",
                            labels_ns="confirmed_labels",
                            r=r,
                            g=g,
                            b=b,
                            alpha=alpha,
                            label_text=label_text,
                            text_color=text_color,
                        )
                    )
                    confirmed_mk.markers.append(leg_cyl)
                    confirmed_mk.markers.append(leg_txt)
                    if self._pub_mk_visible is not None:
                        bucket = confirmed_split_visibility_bucket(
                            is_confirmed=True,
                            is_invalid=False,
                            currently_visible=bool(
                                ent.currently_visible,
                            ),
                            anchor_ok_for_marker=True,
                            publish_split=True,
                        )
                        if bucket == "visible":
                            sv0, sv1 = (
                                self._split_visibility_marker_pair_ids(
                                    ent.entity_id,
                                )
                            )
                            vc, vt = (
                                self._make_confirmed_landmark_marker_pair(
                                    ent,
                                    stamp,
                                    cyl_id=sv0,
                                    txt_id=sv1,
                                    entities_ns="visible_landmark_entities",
                                    labels_ns=(
                                        "visible_landmark_labels"
                                    ),
                                    r=r,
                                    g=g,
                                    b=b,
                                    alpha=alpha,
                                    label_text=label_text,
                                    text_color=text_color,
                                )
                            )
                            visible_mk.markers.append(vc)
                            visible_mk.markers.append(vt)
                        elif bucket == "remembered":
                            sr0, sr1 = (
                                self._split_visibility_marker_pair_ids(
                                    ent.entity_id,
                                )
                            )
                            rc, rt = (
                                self._make_confirmed_landmark_marker_pair(
                                    ent,
                                    stamp,
                                    cyl_id=sr0,
                                    txt_id=sr1,
                                    entities_ns=(
                                        "remembered_landmark_entities"
                                    ),
                                    labels_ns=(
                                        "remembered_landmark_labels"
                                    ),
                                    r=r,
                                    g=g,
                                    b=b,
                                    alpha=alpha,
                                    label_text=label_text,
                                    text_color=text_color,
                                )
                            )
                            remembered_mk.markers.append(rc)
                            remembered_mk.markers.append(rt)
                    skip_plain_cylinder = True
            else:
                if not self._publish_candidates:
                    continue
                alpha = max(0.10, min(0.6, ent.confidence))
                ns_prefix = "candidate"
                target_arr = candidate_mk
                tail = ""
                if ent.candidate_status_hint:
                    tail = (
                        f" candidate_not_confirmed:"
                        f" {ent.candidate_status_hint}"
                    )
                label_text = (
                    f"candidate {short_class} conf={ent.confidence:.2f} "
                    f"n={ent.observations_count}{tail}"
                )
                text_color = ColorRGBA(r=0.65, g=0.85, b=1.00, a=0.85)

            if skip_plain_cylinder:
                continue

            cyl = Marker()
            cyl.header.stamp = stamp
            cyl.header.frame_id = self._frame_id
            cyl.ns = f"{ns_prefix}_entities"
            cyl.id = i * 2 + 0
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position.x = float(ent.px)
            cyl.pose.position.y = float(ent.py)
            cyl.pose.position.z = 0.20
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = 0.30
            cyl.scale.y = 0.30
            cyl.scale.z = 0.40
            cyl.color = ColorRGBA(r=r, g=g, b=b, a=alpha)
            target_arr.markers.append(cyl)

            txt = Marker()
            txt.header.stamp = stamp
            txt.header.frame_id = self._frame_id
            txt.ns = f"{ns_prefix}_labels"
            txt.id = i * 2 + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(ent.px)
            txt.pose.position.y = float(ent.py)
            txt.pose.position.z = float(ent.pz) + 0.40
            if txt.pose.position.z < 0.55:
                txt.pose.position.z = 0.55
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.30
            txt.color = text_color
            txt.text = label_text
            target_arr.markers.append(txt)

        self._pub_mk.publish(confirmed_mk)
        self._pub_dbg_mk.publish(candidate_mk)
        if self._pub_mk_visible is not None:
            self._pub_mk_visible.publish(visible_mk)
            self._pub_mk_remembered.publish(remembered_mk)

    # ------------------------------------------------------------------
    # /semantic_map/island_debug_markers — diagnostic overlay
    # ------------------------------------------------------------------
    def _publish_island_debug(self, stamp: TimeMsg) -> None:
        """Render the most recent island-association attempts.

        Green sphere = snapped point (success).
        Red sphere   = original observation point (rejected).
        Text marker  = rejection reason or accepted island_id.

        Day 9 Task 5 — always publish a status text even when no
        events fired this frame, so the topic is never reduced to
        "DELETEALL forever". The status block summarises:
          * pointcloud anchor success/failure counts so far
          * occupancy island anchor success/failure counts so far
          * map availability + pointcloud buffer health
        which gives the operator a one-glance answer to "why did
        nothing snap to anything?".
        """
        if self._pub_island_dbg_mk is None:
            return
        out = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.stamp = stamp
        clear.header.frame_id = self._frame_id
        clear.ns = "island_debug"
        out.markers.append(clear)

        # Always-present status text. Anchored at the map origin a few
        # metres up so it doesn't collide with semantic markers.
        status_txt = Marker()
        status_txt.header.stamp = stamp
        status_txt.header.frame_id = self._frame_id
        status_txt.ns = "island_debug_status"
        status_txt.id = 0
        status_txt.type = Marker.TEXT_VIEW_FACING
        status_txt.action = Marker.ADD
        status_txt.pose.position.x = 0.0
        status_txt.pose.position.y = 0.0
        status_txt.pose.position.z = 3.0
        status_txt.pose.orientation.w = 1.0
        status_txt.scale.z = 0.30
        c = self._anchor_counters
        status_txt.text = (
            f"anchor stats: obs={c['observations_total']} "
            f"pc_ok={c['pointcloud_anchor_success']} "
            f"isl_ok={c['occupancy_island_anchor_success']} "
            f"none={c['candidate_no_anchor']} "
            f"pc_disagree={c['pc_map_disagreement']} "
            f"map={'Y' if self._island.has_map() else 'N'} "
            f"pc_buf={len(self._pc._frames) if hasattr(self._pc, '_frames') else 0}"
        )
        status_txt.color = ColorRGBA(r=0.85, g=0.85, b=0.95, a=1.0)
        out.markers.append(status_txt)

        for idx, (raw_cls, ox, oy, res) in enumerate(self._last_island_events):
            # Sphere at the relevant point.
            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = self._frame_id
            sphere.ns = "island_debug_points"
            sphere.id = idx * 2 + 0
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            if res.success:
                sphere.pose.position.x = float(res.snapped_x)
                sphere.pose.position.y = float(res.snapped_y)
                sphere.color = ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.8)
            else:
                sphere.pose.position.x = float(ox)
                sphere.pose.position.y = float(oy)
                sphere.color = ColorRGBA(r=0.95, g=0.15, b=0.15, a=0.7)
            sphere.pose.position.z = 0.10
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.18
            sphere.scale.y = 0.18
            sphere.scale.z = 0.18
            out.markers.append(sphere)

            txt = Marker()
            txt.header = sphere.header
            txt.ns = "island_debug_text"
            txt.id = idx * 2 + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = sphere.pose.position.x
            txt.pose.position.y = sphere.pose.position.y
            txt.pose.position.z = 0.30
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.14
            if res.success:
                txt.text = (
                    f"{raw_cls} -> {res.island_id} "
                    f"(n={res.cell_count}, "
                    f"{res.bbox_length_m:.1f}x{res.bbox_width_m:.1f}m)"
                )
                txt.color = ColorRGBA(r=0.5, g=1.0, b=0.5, a=1.0)
            else:
                txt.text = f"{raw_cls} REJECTED: {res.rejection_reason}"
                txt.color = ColorRGBA(r=1.0, g=0.6, b=0.6, a=1.0)
            out.markers.append(txt)

        self._pub_island_dbg_mk.publish(out)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _mint_id(self, cls: str) -> str:
        """Mint a human-readable per-class entity id like ``chair_001``.

        The counter is monotonic across the node's lifetime (does
        NOT reset when an entity is pruned), so a freshly observed
        chair after a prune gets a new id. This makes the RViz
        marker labels intelligible — "chair_001 (n=14)" reads
        better than a random hex string and the operator can spot
        a mid-run identity flip immediately.
        """
        cls_key = cls.replace(" ", "_") or "obj"
        n = self._next_id_per_class.get(cls_key, 0) + 1
        self._next_id_per_class[cls_key] = n
        return f"{cls_key}_{n:03d}"

    def _match_existing(
        self, cls: str, x: float, y: float, z: float
    ) -> Optional[TrackedEntity]:
        """Return the closest same-class entity within `nms_radius_m`,
        or None if none qualifies. Iterates the dict; for the MVP
        cardinality (a handful of entities in a 10 m room) this is
        fine. If you find yourself with >100 entities, swap to a
        spatial index (KD-tree) keyed per class.
        """
        best: Optional[TrackedEntity] = None
        best_d2 = self._nms_r2  # ceiling: anything farther doesn't qualify
        for ent in self._entities.values():
            if ent.class_label != cls:
                continue
            dx = ent.px - x
            dy = ent.py - y
            dz = ent.pz - z
            d2 = dx * dx + dy * dy + dz * dz
            if d2 < best_d2:
                best_d2 = d2
                best = ent
        return best

    def _now_msg(self) -> TimeMsg:
        return self.get_clock().now().to_msg()

    def _tick_log(self) -> None:
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        elapsed_ns = (now - self._last_log_time).nanoseconds
        if elapsed_ns < int(self._log_period * 1e9):
            return
        elapsed_s = elapsed_ns / 1e9
        msg_hz = self._n_messages / elapsed_s if elapsed_s > 0 else 0.0
        self.get_logger().info(
            f"semantic_memory @ {msg_hz:.1f} Hz inputs; "
            f"associations={self._n_associations} "
            f"new_entities={self._n_new_entities} "
            f"alive={len(self._entities)}"
        )
        self._n_messages = 0
        self._n_associations = 0
        self._n_new_entities = 0
        self._last_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMemoryAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
