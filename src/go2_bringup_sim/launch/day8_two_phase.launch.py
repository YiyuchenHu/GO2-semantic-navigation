"""Day 8 — two-phase variant (autonomous mapping + NL command).

A simpler alternative to ``day8.launch.py`` that splits the original
"target-driven exploration" FSM into two clean phases:

    Phase A (autonomous, hands-off):
        mapping_explorer_node drives Go2 around until /map has no
        more frontiers; semantic_memory_aggregator silently records
        chair / table / box / etc as Go2 passes them. publishes
        /mapping/status so the operator knows when DONE.

    Phase B (NL command-driven):
        operator publishes a string on /user_command (e.g.
        "go to chair") via teleop or a one-shot ros2 topic pub.
        nl_parser_node turns it into a SemanticTask on
        /semantic_task/request. task_coordinator (in pure target-driven
        mode) runs the existing Day 7 CHECK_MEMORY → TARGET_FOUND →
        PLAN_APPROACH_GOAL → NAVIGATE_TO_GOAL → ARRIVED path. Because
        the entity was already added to /semantic_map/objects during
        phase A, CHECK_MEMORY hits immediately and the FSM never
        enters EXPLORE — eliminating the consecutive-aborts /
        EXPLORE-stuck failure modes seen in day8.launch.py.

The legacy day8.launch.py is kept as the "single-launch target-driven
EXPLORE" variant. Pick whichever fits the demo: this one is simpler,
more demo-friendly, and recovers cleanly if Go2 needs to be re-tasked.

Pre-requisites — start in this order, in separate terminals
-----------------------------------------------------------
1. Isaac Sim:           bash scripts/run_warehouse_ros2.sh
2. Static TFs + scan:   ros2 launch go2_bringup_sim tf_and_scan.launch.py
3. SLAM + Nav2:         ros2 launch go2_bringup_sim nav2.launch.py slam:=True
4. THIS launch:         ros2 launch go2_bringup_sim day8_two_phase.launch.py
5. RViz (optional):     bash scripts/run_rviz.sh
6. Teleop / commander:  ros2 topic pub --once /user_command \\
                            std_msgs/msg/String "data: 'go to chair'"

What this launch does NOT start
-------------------------------
* Isaac Sim, Nav2, slam_toolbox, tf_and_scan — see steps 1-3 above.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Frame / topic args (kept identical to day7/day8 so RViz / probes
    # need no special config)
    # ------------------------------------------------------------------
    target_frame_arg = DeclareLaunchArgument(
        "target_frame", default_value="map",
        description="Frame for /detections_3d, /semantic_map/objects, "
                    "/get_frontiers' robot_pose, and the Nav2 goal.",
    )
    base_frame_arg = DeclareLaunchArgument(
        "base_frame", default_value="base_link"
    )

    # ------------------------------------------------------------------
    # YOLOE (Day 5) args
    # ------------------------------------------------------------------
    model_path_arg = DeclareLaunchArgument(
        "model_path", default_value="yoloe-11s-seg.pt"
    )
    classes_arg = DeclareLaunchArgument(
        "classes",
        default_value=(
            # MVP demo targets are now person + table. Person is FIRST
            # so YOLOE's text-encoder gives it preferential ranking on
            # mixed-class frames. Synonyms cover Isaac people-pack
            # appearance variations (worker / construction-helmet
            # silhouettes that COCO 'person' was trained on).
            #
            # Table comes with `desk` + `dining table` aliases because
            # YOLOE often identifies the EastRural dining-table USD as
            # `desk`; semantic_memory keeps both class labels distinct,
            # so we inject the alias here AND in the NL parser so a
            # `go to table` user command still resolves a `desk`-tagged
            # entity.
            #
            # Chair stays in the allowlist (the warehouse scene still
            # spawns one) but is no longer the MVP target — too small /
            # thin for clean costmap representation.
            "['person','human','man','worker','people',"
            "'table','desk','dining table','workbench',"
            "'chair','office chair','stool','folding chair','armchair',"
            "'box','crate']"
        ),
    )
    conf_arg = DeclareLaunchArgument("conf_threshold", default_value="0.4")
    iou_arg = DeclareLaunchArgument("iou_threshold", default_value="0.5")
    device_arg = DeclareLaunchArgument("device", default_value="cuda:0")
    half_arg = DeclareLaunchArgument("half", default_value="False")
    publish_overlay_arg = DeclareLaunchArgument(
        "publish_overlay", default_value="True"
    )

    # ------------------------------------------------------------------
    # Depth projector (Day 6) args
    # ------------------------------------------------------------------
    depth_image_arg = DeclareLaunchArgument(
        "depth_image_topic", default_value="/camera/depth/image_rect_raw"
    )
    camera_info_arg = DeclareLaunchArgument(
        "camera_info_topic", default_value="/camera/color/camera_info"
    )
    sync_slop_arg = DeclareLaunchArgument("sync_slop", default_value="0.05")
    tf_timeout_arg = DeclareLaunchArgument(
        "tf_timeout_sec", default_value="1.5"
    )
    # Day 8++ — strict at-stamp TF projection (Tasks 1 + 2). Defaults
    # require depth_projector to look up TF at detection.header.stamp,
    # falling back ONLY to the keyframe pose cache if the live buffer
    # cannot serve that stamp. Latest-TF fallback stays off to stop
    # the "robot rotated mid-frame ⇒ marker lands on a wall" failure.
    use_stamp_tf_arg = DeclareLaunchArgument(
        "use_detection_timestamp_tf", default_value="true",
        description="depth_projector looks up the camera->map TF at "
                    "the detection's header.stamp instead of using "
                    "the latest transform. Required for stable "
                    "semantic markers under fast rotation. Set False "
                    "only for diagnostic A/B testing.",
    )
    allow_latest_tf_fallback_arg = DeclareLaunchArgument(
        "allow_latest_tf_fallback", default_value="false",
        description="Day 8 legacy behaviour: when at-stamp lookup AND "
                    "the keyframe pose cache both fail, use whatever "
                    "transform tf2_ros.Buffer.lookup_transform with "
                    "Time() returns (i.e. latest available). False is "
                    "the safe default; True is for diagnosing TF "
                    "buffer pressure.",
    )
    tf_lookup_timeout_arg = DeclareLaunchArgument(
        "tf_lookup_timeout_sec", default_value="0.2"
    )
    keyframe_cache_age_arg = DeclareLaunchArgument(
        "keyframe_cache_max_age_sec", default_value="2.0"
    )
    max_det_depth_dt_arg = DeclareLaunchArgument(
        "max_detection_depth_dt_sec", default_value="0.2"
    )
    # Legacy alias kept for backwards-compat — depth_projector still
    # OR's it with the new ``allow_latest_tf_fallback``.
    tf_fallback_latest_arg = DeclareLaunchArgument(
        "tf_fallback_latest_on_time_error", default_value="false"
    )
    min_valid_pixels_arg = DeclareLaunchArgument(
        "min_valid_pixels", default_value="30"
    )
    use_masks_arg = DeclareLaunchArgument(
        "use_masks", default_value="True"
    )
    masks_topic_arg = DeclareLaunchArgument(
        "masks_topic", default_value="/detections/masks"
    )
    bbox_shrink_arg = DeclareLaunchArgument(
        "bbox_shrink", default_value="0.20"
    )
    depth_percentile_arg = DeclareLaunchArgument(
        "depth_percentile", default_value="30.0"
    )
    # Day 8++++ — bbox fallback + mask grace + debug stats.
    mask_wait_grace_arg = DeclareLaunchArgument(
        "mask_wait_grace_sec", default_value="0.1",
        description="Day 8++++ Task 1 — when a 3-input (det+depth+info) "
                    "triplet fires without a matching mask in the "
                    "stamp-keyed mask buffer, hold the triplet for up "
                    "to this many seconds before falling back to the "
                    "bbox-center depth path. Larger ⇒ more chance the "
                    "mask catches up; smaller ⇒ snappier 3D output.",
    )
    masks_match_dt_arg = DeclareLaunchArgument(
        "masks_stamp_match_dt_sec", default_value="0.1"
    )
    masks_buf_max_arg = DeclareLaunchArgument(
        "masks_buffer_max_size", default_value="32"
    )
    bbox_fallback_enabled_arg = DeclareLaunchArgument(
        "bbox_fallback_enabled", default_value="true",
        description="Day 8++++ Task 1 — when masks never arrive (or "
                    "arrive empty), sample depth from a tight central "
                    "window inside the YOLOE bbox so /detections_3d "
                    "still produces a rough projection. Disable only "
                    "if you want strict mask-only depth sampling.",
    )
    bbox_fallback_window_arg = DeclareLaunchArgument(
        "bbox_fallback_window_frac", default_value="0.30"
    )
    bbox_fallback_lower_classes_arg = DeclareLaunchArgument(
        "bbox_fallback_lower_center_classes",
        default_value="table desk dining table workbench",
    )
    bbox_fallback_conf_scale_arg = DeclareLaunchArgument(
        "bbox_fallback_confidence_scale", default_value="0.85"
    )
    debug_stats_topic_arg = DeclareLaunchArgument(
        "debug_stats_topic", default_value="/depth_projector/debug_stats"
    )
    debug_stats_period_arg = DeclareLaunchArgument(
        "debug_stats_period_sec", default_value="2.0"
    )

    # ------------------------------------------------------------------
    # Semantic memory args (raised "permanent" knob is the whole reason
    # phase B can rely on phase A's memory).
    # ------------------------------------------------------------------
    nms_radius_arg = DeclareLaunchArgument(
        "nms_radius_m", default_value="0.8"
    )
    position_alpha_arg = DeclareLaunchArgument(
        "position_alpha", default_value="0.3"
    )
    confidence_step_arg = DeclareLaunchArgument(
        "confidence_step_up", default_value="0.15"
    )
    confidence_decay_arg = DeclareLaunchArgument(
        "confidence_decay_rate", default_value="0.02"
    )
    min_det_conf_arg = DeclareLaunchArgument(
        "min_detection_confidence", default_value="0.45",
        description="Reject any YOLOE detection whose confidence is "
                    "below this (per-frame). Raised from the node "
                    "default 0.40 to reduce spurious landmarks while "
                    "still admitting table/desk/score ~0.45–0.55; "
                    "`max_confirmed_table_landmarks:=1` caps table "
                    "duplicates. Tune back to 0.55 only if phantom "
                    "furniture increases.",
    )
    visibility_timeout_arg = DeclareLaunchArgument(
        "visibility_timeout_sec", default_value="5.0"
    )
    permanent_after_obs_arg = DeclareLaunchArgument(
        "permanent_after_observations", default_value="3",
        description="Day 8+: now that semantic_memory snaps every "
                    "observation to an obstacle island (and rejects "
                    "wall-like islands), 3 same-island observations "
                    "≈ 0.2 s of detections are enough to trust an "
                    "entity. Old value (20) was a workaround for "
                    "stale-TF false positives that island association "
                    "now filters out at the source. Combine with "
                    "``island_promotion_count`` and "
                    "``island_promotion_confidence`` for the other "
                    "two promotion paths.",
    )
    entity_merge_radius_arg = DeclareLaunchArgument(
        "entity_merge_radius_m", default_value="1.5"
    )
    # ------------------------------------------------------------------
    # Day 8+ semantic-memory knobs: island association + confirmed
    # landmark persistence + canonical class map. Most parameters keep
    # the in-code defaults (see semantic_memory_aggregator_node.py).
    # ------------------------------------------------------------------
    use_island_assoc_arg = DeclareLaunchArgument(
        "use_occupancy_island_association", default_value="True",
        description="If True, every depth-projected detection is "
                    "snapped to the nearest obstacle island in /map "
                    "before being aggregated. Greatly stabilises "
                    "marker positions (no more dancing 0.5 m around "
                    "the real object) and rejects wall-like detections.",
    )
    island_search_radius_arg = DeclareLaunchArgument(
        "island_search_radius_m", default_value="1.0"
    )
    island_promo_conf_arg = DeclareLaunchArgument(
        "island_promotion_confidence", default_value="0.5"
    )
    island_promo_count_arg = DeclareLaunchArgument(
        "island_promotion_count", default_value="2"
    )
    confirmed_min_conf_arg = DeclareLaunchArgument(
        "confirmed_min_confidence", default_value="0.5"
    )
    keep_confirmed_arg = DeclareLaunchArgument(
        "keep_confirmed_landmarks", default_value="True"
    )
    # ------------------------------------------------------------------
    # Day 8++ wall-rejection + invalidation knobs (Tasks 1 / 2 / 3).
    # Defaults are tuned for person + table MVP targets in a typical
    # warehouse SLAM map (5 cm grid, thick wall lines).
    # ------------------------------------------------------------------
    reject_person_near_wall_arg = DeclareLaunchArgument(
        "reject_person_near_wall", default_value="True",
        description="If True, person observations whose snapped "
                    "island is within `person_min_wall_clearance_m` "
                    "of a wall-like external occupied chain get "
                    "rejected as `person_too_close_to_wall`. "
                    "Stops the 'remembered person on top of a wall' "
                    "false confirmed pattern. Tables can sit closer "
                    "to walls so they have a separate, smaller "
                    "clearance.",
    )
    person_wall_clear_arg = DeclareLaunchArgument(
        "person_min_wall_clearance_m", default_value="0.35"
    )
    table_wall_clear_arg = DeclareLaunchArgument(
        "table_min_wall_clearance_m", default_value="0.15"
    )
    person_max_len_arg = DeclareLaunchArgument(
        "person_max_island_length_m", default_value="1.0"
    )
    table_max_len_arg = DeclareLaunchArgument(
        "table_max_island_length_m", default_value="2.2"
    )
    invalid_thresh_arg = DeclareLaunchArgument(
        "confirmed_invalid_evidence_threshold", default_value="3",
        description="Number of repeated bad-evidence events "
                    "(wall_like / too_close_to_wall / shape_invalid / "
                    "outside_map / unknown_cell) needed before a "
                    "previously-confirmed landmark is retired as "
                    "is_invalid=True. target_selector skips invalid "
                    "entities; markers reroute to "
                    "/semantic_map/debug_markers so RViz still shows "
                    "the operator that something used to be there.",
    )
    allow_invalidation_arg = DeclareLaunchArgument(
        "allow_confirmed_invalidation", default_value="True"
    )
    # ------------------------------------------------------------------
    # Day 8++ Tasks 3–5 — duplicate person merge + per-class promotion
    # gates + island-required publication / selection filter.
    # ------------------------------------------------------------------
    merge_person_radius_arg = DeclareLaunchArgument(
        "merge_person_radius_m", default_value="1.5",
        description="Two same-class person entities within this "
                    "geometric radius get fused into one. MVP "
                    "warehouse has only ONE real person, so this "
                    "should be aggressive.",
    )
    merge_table_radius_arg = DeclareLaunchArgument(
        "merge_table_radius_m", default_value="2.5",
        description="Warehouse table MVP: widen slightly so jittered "
                    "YOLOE centroids converge within a few housekeeping "
                    "ticks; capped by max_confirmed_table_landmarks=1.",
    )
    max_confirmed_person_arg = DeclareLaunchArgument(
        "max_confirmed_person_landmarks", default_value="1",
        description="Per-class cap on confirmed entities. >0 ⇒ all "
                    "but the top-N (by quality_score) get demoted "
                    "back to candidate so the marker stream cannot "
                    "grow stale duplicates. 1 = MVP one-person demo.",
    )
    max_confirmed_table_arg = DeclareLaunchArgument(
        "max_confirmed_table_landmarks", default_value="1",
        description="Day 9 default raised from 0 (off) to 1 (cap). "
                    "RViz routinely showed 3-4 ghost table markers "
                    "from oblique YOLOE detections; capping to 1 "
                    "matches the single-table MVP scene.",
    )
    person_min_obs_arg = DeclareLaunchArgument(
        "person_min_observations_to_confirm", default_value="2",
        description="Person-specific promotion gate. Combined with "
                    "the island fast-path: a single high-confidence "
                    "detection on a wall fragment can no longer "
                    "create a confirmed person landmark.",
    )
    table_min_obs_arg = DeclareLaunchArgument(
        "table_min_observations_to_confirm", default_value="2",
        description="Table-specific promotion gate: remembered "
                    "landmarks need more evidence than flaky "
                    "candidates; combines with LiDAR/isl anchoring.",
    )
    single_obs_island_classes_arg = DeclareLaunchArgument(
        "allow_single_observation_island_promotion_classes",
        default_value="",
        description="Empty disables single-frame island promo for "
                    "every class — table confirms via recurrence / "
                    "observation count instead.",
    )
    require_island_classes_arg = DeclareLaunchArgument(
        "require_island_anchor_for_classes", default_value="person table",
        description="Space-separated classes that MUST have a pc_ or isl_ "
                    "anchor before promotion + /semantic_map/markers. "
                    "Keeps remembered stream clean.",
    )
    mark_unanchored_invalid_arg = DeclareLaunchArgument(
        "mark_unanchored_required_classes_invalid", default_value="true",
        description="Day 8++++ Task 3 — when housekeeping finds a "
                    "confirmed entity of a require_island_anchor class "
                    "with no island_id, mark it INVALID (true) or just "
                    "demote to candidate (false). True is the strict "
                    "default: invalid entities are red in RViz and "
                    "cannot be selected, making the false positive "
                    "obvious to the operator.",
    )
    # ------------------------------------------------------------------
    # Day 9 — PointCloud2 cluster anchor + anchor stats publication.
    # The cluster anchor is the new primary geometry source for
    # semantic landmarks; occupancy island remains as cross-validation.
    # ------------------------------------------------------------------
    use_pc_anchor_arg = DeclareLaunchArgument(
        "use_pointcloud_cluster_anchor", default_value="true",
        description="Day 9 Task 1 — enable LiDAR PointCloud2 cluster "
                    "anchoring for semantic landmarks. Primary geometry "
                    "source for person/table; /map island anchoring "
                    "drops to cross-validation when this is on.",
    )
    pc_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic", default_value="/lidar/points",
        description="Topic feeding the PointCloud cluster associator. "
                    "Set to /camera/depth/points to fall back on the "
                    "depth-camera cloud when LiDAR is offline.",
    )
    pc_default_radius_arg = DeclareLaunchArgument(
        "pointcloud_anchor_search_radius_m", default_value="1.0",
        description="Default class-agnostic search radius for the PC "
                    "cluster associator. Tuned per class via the "
                    "person_/table_pointcloud_search_radius_m args.",
    )
    pc_person_radius_arg = DeclareLaunchArgument(
        "person_pointcloud_search_radius_m", default_value="1.2",
    )
    pc_table_radius_arg = DeclareLaunchArgument(
        "table_pointcloud_search_radius_m", default_value="1.5",
    )
    pc_min_pts_arg = DeclareLaunchArgument(
        "pointcloud_min_cluster_points", default_value="5",
    )
    pc_max_pts_arg = DeclareLaunchArgument(
        "pointcloud_max_cluster_points", default_value="5000",
    )
    pc_tol_arg = DeclareLaunchArgument(
        "pointcloud_cluster_tolerance_m", default_value="0.20",
    )
    pc_person_zmin_arg = DeclareLaunchArgument(
        "person_pointcloud_z_min", default_value="0.05",
    )
    pc_person_zmax_arg = DeclareLaunchArgument(
        "person_pointcloud_z_max", default_value="1.9",
    )
    pc_table_zmin_arg = DeclareLaunchArgument(
        "table_pointcloud_z_min", default_value="0.05",
    )
    pc_table_zmax_arg = DeclareLaunchArgument(
        "table_pointcloud_z_max", default_value="1.3",
    )
    pc_buffer_arg = DeclareLaunchArgument(
        "pointcloud_buffer_size", default_value="4",
    )
    pc_max_per_cloud_arg = DeclareLaunchArgument(
        "pointcloud_max_points_per_cloud", default_value="20000",
    )
    anchor_stats_topic_arg = DeclareLaunchArgument(
        "anchor_debug_stats_topic",
        default_value="/semantic_map/anchor_debug_stats",
    )
    anchor_stats_period_arg = DeclareLaunchArgument(
        "anchor_debug_stats_period_sec", default_value="2.0",
    )
    # ------------------------------------------------------------------
    # Day 9 Task 7 — frontier semantic / island keep-out.
    # ------------------------------------------------------------------
    frontier_reject_inside_islands_arg = DeclareLaunchArgument(
        "reject_frontiers_inside_obstacle_islands", default_value="true",
        description="Drop frontier centroids that sit inside (or within "
                    "obstacle_island_inflation_m of) an occupied island.",
    )
    frontier_semantic_clearance_arg = DeclareLaunchArgument(
        "frontier_min_clearance_from_semantic_obstacles_m",
        default_value="0.8",
    )
    frontier_reject_unknown_pockets_arg = DeclareLaunchArgument(
        "frontier_reject_unknown_pockets", default_value="true",
    )
    obstacle_island_inflation_arg = DeclareLaunchArgument(
        "obstacle_island_inflation_m", default_value="0.5",
    )

    # ------------------------------------------------------------------
    # Day 7 target_selector / approach_planner args (unchanged from
    # day7.launch.py — they're driven by SemanticTask now, not by a
    # launch-time target_class default).
    # ------------------------------------------------------------------
    selector_min_conf_arg = DeclareLaunchArgument(
        "selector_min_confidence", default_value="0.20"
    )
    selector_period_arg = DeclareLaunchArgument(
        "selector_period_sec", default_value="0.5"
    )
    selector_dist_unknown_pen_arg = DeclareLaunchArgument(
        "selector_distance_unknown_penalty", default_value="0.5",
        description="target_selector applies this penalty to the "
                    "score when map->base_link TF is unavailable, so "
                    "a candidate whose distance we cannot even "
                    "compute loses to one we can.",
    )
    selector_reject_dist_unknown_arg = DeclareLaunchArgument(
        "selector_reject_if_distance_unknown", default_value="false",
        description="Hard-reject all candidates whenever map->base_link "
                    "TF is unavailable instead of penalising. Useful "
                    "for production demos that should refuse to ship "
                    "a target without a verified robot pose.",
    )
    costmap_topic_arg = DeclareLaunchArgument(
        "costmap_topic", default_value="/global_costmap/costmap"
    )
    nav_action_arg = DeclareLaunchArgument(
        "nav_action_name", default_value="/navigate_to_pose"
    )
    num_samples_arg = DeclareLaunchArgument(
        "num_angle_samples", default_value="16"
    )
    approach_dist_default_arg = DeclareLaunchArgument(
        "approach_distance_default", default_value="0.9"
    )
    approach_dist_chair_arg = DeclareLaunchArgument(
        "approach_distance_chair", default_value="0.9"
    )
    approach_dist_table_arg = DeclareLaunchArgument(
        "approach_distance_table", default_value="1.0"
    )
    approach_dist_desk_arg = DeclareLaunchArgument(
        "approach_distance_desk", default_value="1.0"
    )
    approach_dist_box_arg = DeclareLaunchArgument(
        "approach_distance_box", default_value="0.7"
    )
    approach_dist_person_arg = DeclareLaunchArgument(
        "approach_distance_person", default_value="1.2"
    )
    cost_threshold_arg = DeclareLaunchArgument(
        "cost_threshold", default_value="60"
    )
    replan_period_arg = DeclareLaunchArgument(
        "replan_period_sec", default_value="1.0"
    )
    replan_distance_arg = DeclareLaunchArgument(
        "replan_distance_m", default_value="0.10"
    )

    # ------------------------------------------------------------------
    # Frontier explorer args (same as day8.launch.py)
    # ------------------------------------------------------------------
    map_topic_arg = DeclareLaunchArgument(
        "map_topic", default_value="/map"
    )
    min_cluster_size_arg = DeclareLaunchArgument(
        "min_cluster_size", default_value="10"
    )
    info_gain_radius_arg = DeclareLaunchArgument(
        "info_gain_radius_m", default_value="1.5"
    )
    distance_weight_arg = DeclareLaunchArgument(
        "distance_weight", default_value="5.0"
    )
    max_frontiers_arg = DeclareLaunchArgument(
        "max_frontiers", default_value="5"
    )
    safety_radius_arg = DeclareLaunchArgument(
        "safety_radius_m", default_value="0.4"
    )
    snap_search_radius_arg = DeclareLaunchArgument(
        "snap_search_radius_m", default_value="1.0"
    )
    cm_safe_max_cost_arg = DeclareLaunchArgument(
        "costmap_safe_max_cost", default_value="75"
    )

    # Frontier bounding-box filter (map frame). The Isaac Sim warehouse
    # spans [-5, +5] × [-5, +5] in world; tf_and_scan.launch.py shifts
    # the world origin into map by (-4, -4), so the warehouse occupies
    # roughly [-1, 9] × [-1, 9] in MAP frame. We bound the bbox a bit
    # tighter than the wall coordinates so a frontier centroid sitting
    # right against a wall can still be inflated/snapped without
    # crossing the bbox edge. May-8 ran into 14.99 m frontiers out
    # past the east wall — see the comment in frontier_explorer_node.py
    # where the bbox parameters are declared for the full story.
    bbox_xmin_arg = DeclareLaunchArgument(
        "frontier_bbox_xmin", default_value="-1.5",
        description="Map-frame X lower bound (m) for valid frontier "
                    "centroids. Anything below is rejected.",
    )
    bbox_ymin_arg = DeclareLaunchArgument(
        "frontier_bbox_ymin", default_value="-1.5",
        description="Map-frame Y lower bound (m) for valid frontier "
                    "centroids.",
    )
    bbox_xmax_arg = DeclareLaunchArgument(
        "frontier_bbox_xmax", default_value="9.5",
        description="Map-frame X upper bound (m) for valid frontier "
                    "centroids.",
    )
    bbox_ymax_arg = DeclareLaunchArgument(
        "frontier_bbox_ymax", default_value="9.5",
        description="Map-frame Y upper bound (m) for valid frontier "
                    "centroids.",
    )

    # ------------------------------------------------------------------
    # mapping_explorer args
    # ------------------------------------------------------------------
    map_done_confirm_arg = DeclareLaunchArgument(
        "map_done_confirm_sec", default_value="5.0",
        description="mapping_explorer holds DONE for this many seconds "
                    "of empty-frontier responses before locking it. "
                    "Avoids flapping when SLAM briefly shows zero "
                    "frontiers between scans.",
    )
    map_max_aborts_arg = DeclareLaunchArgument(
        "map_max_aborts", default_value="4",
        description="Skip up to N consecutive frontier nav-aborts "
                    "before declaring mapping FAILED. Higher than "
                    "task_coordinator's default 3 because we'd rather "
                    "skip a bad goal than abort the entire sweep.",
    )
    abort_cooldown_arg = DeclareLaunchArgument(
        "abort_cooldown_sec", default_value="15.0",
        description="Per-frontier soft-skip window (seconds) after a "
                    "Nav2 ABORT. Lower = Go2 retries failed frontiers "
                    "faster (less idle 'thinking' time); higher = less "
                    "thrash on transient costmap-inflation failures. "
                    "Default 15 s; the original day8 default was 30 s.",
    )

    # ------------------------------------------------------------------
    # task_coordinator args — pure target-driven, NO default class.
    # ------------------------------------------------------------------
    coord_log_period_arg = DeclareLaunchArgument(
        "coord_log_period_sec", default_value="2.0"
    )
    coord_tick_period_arg = DeclareLaunchArgument(
        "coord_tick_period_sec", default_value="0.2"
    )

    # ------------------------------------------------------------------
    # NL parser args
    # ------------------------------------------------------------------
    nl_known_classes_arg = DeclareLaunchArgument(
        "nl_known_classes",
        # Day 8+ MVP targets: person + table FIRST. Order matters for
        # nl_parser's tier-1 exact-match: when the user types
        # "go to person" we want the parser's tokenizer to land on
        # "person" before "human"/"man" trip the alias path. `desk`
        # stays in known_classes because YOLOE often emits `desk` for
        # the EastRural dining table — the synonym table maps it to
        # `table` so target_selector still resolves the right entity.
        default_value="['person', 'table', 'desk', 'chair', 'box']",
        description="Whitelist of classes the operator can navigate to. "
                    "Must be a subset of YOLOE's `classes` arg above. "
                    "Keep tight — every extra class widens the surface "
                    "for fuzzy false-positives.",
    )
    nl_min_conf_arg = DeclareLaunchArgument(
        "nl_min_match_confidence", default_value="0.65"
    )

    # ------------------------------------------------------------------
    # Substitutions
    # ------------------------------------------------------------------
    target_frame = LaunchConfiguration("target_frame")
    base_frame = LaunchConfiguration("base_frame")
    model_path = LaunchConfiguration("model_path")
    classes = LaunchConfiguration("classes")
    conf = LaunchConfiguration("conf_threshold")
    iou = LaunchConfiguration("iou_threshold")
    device = LaunchConfiguration("device")
    half = LaunchConfiguration("half")
    publish_overlay = LaunchConfiguration("publish_overlay")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    sync_slop = LaunchConfiguration("sync_slop")
    tf_timeout_sec = LaunchConfiguration("tf_timeout_sec")
    tf_fallback_latest_on_time_error = LaunchConfiguration(
        "tf_fallback_latest_on_time_error"
    )
    use_detection_timestamp_tf = LaunchConfiguration(
        "use_detection_timestamp_tf"
    )
    allow_latest_tf_fallback = LaunchConfiguration(
        "allow_latest_tf_fallback"
    )
    tf_lookup_timeout_sec = LaunchConfiguration("tf_lookup_timeout_sec")
    keyframe_cache_max_age_sec = LaunchConfiguration(
        "keyframe_cache_max_age_sec"
    )
    max_detection_depth_dt_sec = LaunchConfiguration(
        "max_detection_depth_dt_sec"
    )
    min_valid_pixels = LaunchConfiguration("min_valid_pixels")
    use_masks = LaunchConfiguration("use_masks")
    masks_topic = LaunchConfiguration("masks_topic")
    bbox_shrink = LaunchConfiguration("bbox_shrink")
    depth_percentile = LaunchConfiguration("depth_percentile")
    mask_wait_grace_sec = LaunchConfiguration("mask_wait_grace_sec")
    masks_stamp_match_dt_sec = LaunchConfiguration("masks_stamp_match_dt_sec")
    masks_buffer_max_size = LaunchConfiguration("masks_buffer_max_size")
    bbox_fallback_enabled = LaunchConfiguration("bbox_fallback_enabled")
    bbox_fallback_window_frac = LaunchConfiguration("bbox_fallback_window_frac")
    bbox_fallback_lower_center_classes = LaunchConfiguration(
        "bbox_fallback_lower_center_classes"
    )
    bbox_fallback_confidence_scale = LaunchConfiguration(
        "bbox_fallback_confidence_scale"
    )
    debug_stats_topic = LaunchConfiguration("debug_stats_topic")
    debug_stats_period_sec = LaunchConfiguration("debug_stats_period_sec")
    nms_radius_m = LaunchConfiguration("nms_radius_m")
    position_alpha = LaunchConfiguration("position_alpha")
    confidence_step_up = LaunchConfiguration("confidence_step_up")
    confidence_decay_rate = LaunchConfiguration("confidence_decay_rate")
    min_detection_confidence = LaunchConfiguration("min_detection_confidence")
    visibility_timeout_sec = LaunchConfiguration("visibility_timeout_sec")
    permanent_after_observations = LaunchConfiguration(
        "permanent_after_observations"
    )
    entity_merge_radius_m = LaunchConfiguration("entity_merge_radius_m")
    use_occupancy_island_association = LaunchConfiguration(
        "use_occupancy_island_association"
    )
    island_search_radius_m = LaunchConfiguration("island_search_radius_m")
    island_promotion_confidence = LaunchConfiguration(
        "island_promotion_confidence"
    )
    island_promotion_count = LaunchConfiguration("island_promotion_count")
    confirmed_min_confidence = LaunchConfiguration("confirmed_min_confidence")
    keep_confirmed_landmarks = LaunchConfiguration("keep_confirmed_landmarks")
    reject_person_near_wall = LaunchConfiguration("reject_person_near_wall")
    person_min_wall_clearance_m = LaunchConfiguration(
        "person_min_wall_clearance_m"
    )
    table_min_wall_clearance_m = LaunchConfiguration(
        "table_min_wall_clearance_m"
    )
    person_max_island_length_m = LaunchConfiguration(
        "person_max_island_length_m"
    )
    table_max_island_length_m = LaunchConfiguration(
        "table_max_island_length_m"
    )
    confirmed_invalid_evidence_threshold = LaunchConfiguration(
        "confirmed_invalid_evidence_threshold"
    )
    allow_confirmed_invalidation = LaunchConfiguration(
        "allow_confirmed_invalidation"
    )
    merge_person_radius_m = LaunchConfiguration("merge_person_radius_m")
    merge_table_radius_m = LaunchConfiguration("merge_table_radius_m")
    max_confirmed_person_landmarks = LaunchConfiguration(
        "max_confirmed_person_landmarks"
    )
    max_confirmed_table_landmarks = LaunchConfiguration(
        "max_confirmed_table_landmarks"
    )
    person_min_observations_to_confirm = LaunchConfiguration(
        "person_min_observations_to_confirm"
    )
    table_min_observations_to_confirm = LaunchConfiguration(
        "table_min_observations_to_confirm"
    )
    allow_single_observation_island_promotion_classes = LaunchConfiguration(
        "allow_single_observation_island_promotion_classes"
    )
    require_island_anchor_for_classes = LaunchConfiguration(
        "require_island_anchor_for_classes"
    )
    mark_unanchored_required_classes_invalid = LaunchConfiguration(
        "mark_unanchored_required_classes_invalid"
    )
    # Day 9 — pointcloud anchor + frontier keep-out launch handles.
    use_pointcloud_cluster_anchor = LaunchConfiguration(
        "use_pointcloud_cluster_anchor"
    )
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    pointcloud_anchor_search_radius_m = LaunchConfiguration(
        "pointcloud_anchor_search_radius_m"
    )
    person_pointcloud_search_radius_m = LaunchConfiguration(
        "person_pointcloud_search_radius_m"
    )
    table_pointcloud_search_radius_m = LaunchConfiguration(
        "table_pointcloud_search_radius_m"
    )
    pointcloud_min_cluster_points = LaunchConfiguration(
        "pointcloud_min_cluster_points"
    )
    pointcloud_max_cluster_points = LaunchConfiguration(
        "pointcloud_max_cluster_points"
    )
    pointcloud_cluster_tolerance_m = LaunchConfiguration(
        "pointcloud_cluster_tolerance_m"
    )
    person_pointcloud_z_min = LaunchConfiguration(
        "person_pointcloud_z_min"
    )
    person_pointcloud_z_max = LaunchConfiguration(
        "person_pointcloud_z_max"
    )
    table_pointcloud_z_min = LaunchConfiguration(
        "table_pointcloud_z_min"
    )
    table_pointcloud_z_max = LaunchConfiguration(
        "table_pointcloud_z_max"
    )
    pointcloud_buffer_size = LaunchConfiguration(
        "pointcloud_buffer_size"
    )
    pointcloud_max_points_per_cloud = LaunchConfiguration(
        "pointcloud_max_points_per_cloud"
    )
    anchor_debug_stats_topic = LaunchConfiguration(
        "anchor_debug_stats_topic"
    )
    anchor_debug_stats_period_sec = LaunchConfiguration(
        "anchor_debug_stats_period_sec"
    )
    reject_frontiers_inside_obstacle_islands = LaunchConfiguration(
        "reject_frontiers_inside_obstacle_islands"
    )
    frontier_min_clearance_from_semantic_obstacles_m = LaunchConfiguration(
        "frontier_min_clearance_from_semantic_obstacles_m"
    )
    frontier_reject_unknown_pockets = LaunchConfiguration(
        "frontier_reject_unknown_pockets"
    )
    obstacle_island_inflation_m = LaunchConfiguration(
        "obstacle_island_inflation_m"
    )
    selector_distance_unknown_penalty = LaunchConfiguration(
        "selector_distance_unknown_penalty"
    )
    selector_reject_if_distance_unknown = LaunchConfiguration(
        "selector_reject_if_distance_unknown"
    )
    selector_min_confidence = LaunchConfiguration("selector_min_confidence")
    selector_period_sec = LaunchConfiguration("selector_period_sec")
    costmap_topic = LaunchConfiguration("costmap_topic")
    nav_action_name = LaunchConfiguration("nav_action_name")
    num_angle_samples = LaunchConfiguration("num_angle_samples")
    approach_distance_default = LaunchConfiguration(
        "approach_distance_default"
    )
    approach_distance_chair = LaunchConfiguration("approach_distance_chair")
    approach_distance_table = LaunchConfiguration("approach_distance_table")
    approach_distance_desk = LaunchConfiguration("approach_distance_desk")
    approach_distance_box = LaunchConfiguration("approach_distance_box")
    approach_distance_person = LaunchConfiguration(
        "approach_distance_person"
    )
    cost_threshold = LaunchConfiguration("cost_threshold")
    replan_period_sec = LaunchConfiguration("replan_period_sec")
    replan_distance_m = LaunchConfiguration("replan_distance_m")
    map_topic = LaunchConfiguration("map_topic")
    min_cluster_size = LaunchConfiguration("min_cluster_size")
    info_gain_radius_m = LaunchConfiguration("info_gain_radius_m")
    distance_weight = LaunchConfiguration("distance_weight")
    max_frontiers = LaunchConfiguration("max_frontiers")
    safety_radius_m = LaunchConfiguration("safety_radius_m")
    snap_search_radius_m = LaunchConfiguration("snap_search_radius_m")
    costmap_safe_max_cost = LaunchConfiguration("costmap_safe_max_cost")
    frontier_bbox_xmin = LaunchConfiguration("frontier_bbox_xmin")
    frontier_bbox_ymin = LaunchConfiguration("frontier_bbox_ymin")
    frontier_bbox_xmax = LaunchConfiguration("frontier_bbox_xmax")
    frontier_bbox_ymax = LaunchConfiguration("frontier_bbox_ymax")
    map_done_confirm_sec = LaunchConfiguration("map_done_confirm_sec")
    map_max_aborts = LaunchConfiguration("map_max_aborts")
    abort_cooldown_sec = LaunchConfiguration("abort_cooldown_sec")
    coord_log_period_sec = LaunchConfiguration("coord_log_period_sec")
    coord_tick_period_sec = LaunchConfiguration("coord_tick_period_sec")
    nl_known_classes = LaunchConfiguration("nl_known_classes")
    nl_min_match_confidence = LaunchConfiguration("nl_min_match_confidence")

    # ------------------------------------------------------------------
    # Nodes — perception (Day 6/7 stack, identical to day7.launch.py)
    # ------------------------------------------------------------------
    yoloe_node = Node(
        package="go2_perception",
        executable="yoloe_detector_node",
        name="yoloe_detector",
        output="screen",
        parameters=[{
            "model_path": model_path,
            "classes": classes,
            "conf_threshold": conf,
            "iou_threshold": iou,
            "device": device,
            "half": half,
            "publish_overlay": publish_overlay,
            "input_topic": "/camera/color/image_raw",
            "masks_topic": masks_topic,
            "log_period_sec": 5.0,
        }],
    )
    depth_projector = Node(
        package="go2_semantic_perception",
        executable="depth_projector_node",
        name="depth_projector",
        output="screen",
        parameters=[{
            "detections_topic": "/detections",
            "masks_topic": masks_topic,
            "use_masks": use_masks,
            "depth_image_topic": depth_image_topic,
            "camera_info_topic": camera_info_topic,
            "output_topic": "/detections_3d",
            "target_frame": target_frame,
            "sync_slop": sync_slop,
            "tf_timeout_sec": tf_timeout_sec,
            "tf_fallback_latest_on_time_error": (
                tf_fallback_latest_on_time_error
            ),
            "use_detection_timestamp_tf": use_detection_timestamp_tf,
            "allow_latest_tf_fallback": allow_latest_tf_fallback,
            "tf_lookup_timeout_sec": tf_lookup_timeout_sec,
            "keyframe_cache_max_age_sec": keyframe_cache_max_age_sec,
            "max_detection_depth_dt_sec": max_detection_depth_dt_sec,
            "min_valid_pixels": min_valid_pixels,
            "bbox_shrink": bbox_shrink,
            "depth_percentile": depth_percentile,
            # Day 8++++ — bbox fallback + mask grace + debug stats.
            "mask_wait_grace_sec": mask_wait_grace_sec,
            "masks_stamp_match_dt_sec": masks_stamp_match_dt_sec,
            "masks_buffer_max_size": masks_buffer_max_size,
            "bbox_fallback_enabled": bbox_fallback_enabled,
            "bbox_fallback_window_frac": bbox_fallback_window_frac,
            "bbox_fallback_lower_center_classes": (
                bbox_fallback_lower_center_classes
            ),
            "bbox_fallback_confidence_scale": (
                bbox_fallback_confidence_scale
            ),
            "debug_stats_topic": debug_stats_topic,
            "debug_stats_period_sec": debug_stats_period_sec,
        }],
    )
    semantic_memory = Node(
        package="go2_semantic_perception",
        executable="semantic_memory_aggregator_node",
        name="semantic_memory_aggregator",
        output="screen",
        parameters=[{
            "detections_3d_topic": "/detections_3d",
            "objects_topic": "/semantic_map/objects",
            "markers_topic": "/semantic_map/markers",
            "debug_markers_topic": "/semantic_map/debug_markers",
            "island_debug_markers_topic":
                "/semantic_map/island_debug_markers",
            "map_topic": map_topic,
            "frame_id": target_frame,
            "nms_radius_m": nms_radius_m,
            "position_alpha": position_alpha,
            "confidence_step_up": confidence_step_up,
            "confidence_decay_rate": confidence_decay_rate,
            "min_detection_confidence": min_detection_confidence,
            "visibility_timeout_sec": visibility_timeout_sec,
            "permanent_after_observations": permanent_after_observations,
            "entity_merge_radius_m": entity_merge_radius_m,
            # Day 8+ island association + persistent confirmed
            # landmarks. Most knobs use in-code defaults; we only
            # surface the few an operator typically wants to tune.
            "use_occupancy_island_association":
                use_occupancy_island_association,
            "island_search_radius_m": island_search_radius_m,
            "island_promotion_confidence": island_promotion_confidence,
            "island_promotion_count": island_promotion_count,
            "confirmed_min_confidence": confirmed_min_confidence,
            "keep_confirmed_landmarks": keep_confirmed_landmarks,
            # Day 8++ class-specific shape + wall-clearance overrides
            # (Tasks 1 / 2). Other class knobs (max_island_cells,
            # max_width_m, aspect, min_island_cells) keep their
            # in-code defaults; expose them here when an operator
            # actually needs to override.
            "reject_person_near_wall": reject_person_near_wall,
            "person_min_wall_clearance_m": person_min_wall_clearance_m,
            "table_min_wall_clearance_m": table_min_wall_clearance_m,
            "person_max_island_length_m": person_max_island_length_m,
            "table_max_island_length_m": table_max_island_length_m,
            # Day 8++ confirmed-landmark invalidation (Task 3).
            "allow_confirmed_invalidation": allow_confirmed_invalidation,
            "confirmed_invalid_evidence_threshold":
                confirmed_invalid_evidence_threshold,
            # Day 8++ Tasks 3 / 4 / 5 — duplicate person merging,
            # per-class promotion gates, island-required publishing.
            "merge_person_radius_m": merge_person_radius_m,
            "merge_table_radius_m": merge_table_radius_m,
            "max_confirmed_person_landmarks":
                max_confirmed_person_landmarks,
            "max_confirmed_table_landmarks":
                max_confirmed_table_landmarks,
            "person_min_observations_to_confirm":
                person_min_observations_to_confirm,
            "table_min_observations_to_confirm":
                table_min_observations_to_confirm,
            "allow_single_observation_island_promotion_classes":
                allow_single_observation_island_promotion_classes,
            "require_island_anchor_for_classes":
                require_island_anchor_for_classes,
            "mark_unanchored_required_classes_invalid":
                mark_unanchored_required_classes_invalid,
            # Day 9 — PointCloud2 cluster anchoring + anchor stats.
            "use_pointcloud_cluster_anchor":
                use_pointcloud_cluster_anchor,
            "pointcloud_topic": pointcloud_topic,
            "pointcloud_anchor_search_radius_m":
                pointcloud_anchor_search_radius_m,
            "person_pointcloud_search_radius_m":
                person_pointcloud_search_radius_m,
            "table_pointcloud_search_radius_m":
                table_pointcloud_search_radius_m,
            "pointcloud_min_cluster_points":
                pointcloud_min_cluster_points,
            "pointcloud_max_cluster_points":
                pointcloud_max_cluster_points,
            "pointcloud_cluster_tolerance_m":
                pointcloud_cluster_tolerance_m,
            "person_pointcloud_z_min": person_pointcloud_z_min,
            "person_pointcloud_z_max": person_pointcloud_z_max,
            "table_pointcloud_z_min": table_pointcloud_z_min,
            "table_pointcloud_z_max": table_pointcloud_z_max,
            "pointcloud_buffer_size": pointcloud_buffer_size,
            "pointcloud_max_points_per_cloud":
                pointcloud_max_points_per_cloud,
            "anchor_debug_stats_topic": anchor_debug_stats_topic,
            "anchor_debug_stats_period_sec":
                anchor_debug_stats_period_sec,
        }],
    )

    # ------------------------------------------------------------------
    # Phase A — frontier_explorer + mapping_explorer
    # ------------------------------------------------------------------
    frontier_node = Node(
        package="go2_navigation",
        executable="frontier_explorer_node",
        name="frontier_explorer",
        output="screen",
        parameters=[{
            "map_topic": map_topic,
            "min_cluster_size": min_cluster_size,
            "info_gain_radius_m": info_gain_radius_m,
            "distance_weight": distance_weight,
            "max_frontiers": max_frontiers,
            "safety_radius_m": safety_radius_m,
            "snap_search_radius_m": snap_search_radius_m,
            "costmap_topic": costmap_topic,
            "costmap_safe_max_cost": costmap_safe_max_cost,
            "bbox_xmin": frontier_bbox_xmin,
            "bbox_ymin": frontier_bbox_ymin,
            "bbox_xmax": frontier_bbox_xmax,
            "bbox_ymax": frontier_bbox_ymax,
            "marker_topic": "/frontier_markers",
            "marker_ns": "frontiers",
            # Day 9 Task 7 — keep-out filter against obstacle islands
            # / unknown pockets / semantic landmarks. Stops Go2 from
            # picking a goal under the table.
            "reject_frontiers_inside_obstacle_islands":
                reject_frontiers_inside_obstacle_islands,
            "frontier_min_clearance_from_semantic_obstacles_m":
                frontier_min_clearance_from_semantic_obstacles_m,
            "frontier_reject_unknown_pockets":
                frontier_reject_unknown_pockets,
            "obstacle_island_inflation_m": obstacle_island_inflation_m,
        }],
    )
    mapping_explorer = Node(
        package="go2_navigation",
        executable="mapping_explorer_node",
        name="mapping_explorer",
        output="screen",
        parameters=[{
            "global_frame": target_frame,
            "base_frame": base_frame,
            "get_frontiers_service": "/get_frontiers",
            "nav_action_name": nav_action_name,
            "status_topic": "/mapping/status",
            "control_topic": "/mapping/control",
            "tick_period_sec": 0.5,
            "log_period_sec": 5.0,
            "max_consecutive_aborts": map_max_aborts,
            "done_confirm_sec": map_done_confirm_sec,
            "done_fast": False,
            "abort_cooldown_sec": abort_cooldown_sec,
        }],
    )

    # ------------------------------------------------------------------
    # Phase B — target_selector + approach_planner + task_coordinator
    #          + nl_parser
    # task_coordinator runs in target-driven mode ONLY (no
    # default_target_class, no parse_command_fallback). It will sit in
    # IDLE until nl_parser publishes a SemanticTask, then walk the
    # standard CHECK_MEMORY → ARRIVED path. It NEVER enters EXPLORE
    # because semantic_memory is already populated by phase A.
    # ------------------------------------------------------------------
    target_selector = Node(
        package="go2_semantic_perception",
        executable="target_selector_node",
        name="target_selector",
        output="screen",
        parameters=[{
            "entities_topic": "/semantic_map/objects",
            "selected_topic": "/target/selected",
            # Empty target_class: target_selector waits for an external
            # `ros2 param set /target_selector target_class <cls>`.
            # task_coordinator does this implicitly via approach_planner.
            "target_class": "",
            "min_confidence": selector_min_confidence,
            "select_period_sec": selector_period_sec,
            "base_frame": base_frame,
            "global_frame": target_frame,
            # Day 8+: prefer confirmed (island-anchored or
            # multi-observation) landmarks. Threshold matches
            # ``permanent_after_observations``.
            "confirmed_observations_threshold":
                permanent_after_observations,
            "score_weight_confirmed": 1.5,
            "min_observations_count": 1,
            # MVP keeps require_confirmed=False so that early in
            # Phase A even a fresh candidate person can be picked;
            # set True for production demos that should refuse to
            # navigate to a one-frame ghost.
            "require_confirmed_for_target": False,
            # Day 8++ Task 5 — refuse to select e.g. a confirmed
            # person without an island anchor. Mirrors the aggregator
            # publish gate so the two stay in sync.
            "require_island_anchor_for_classes":
                require_island_anchor_for_classes,
            # Day 8++ Task 6 — never silently set distance to 0 when
            # map->base_link TF fails. Penalise instead, log
            # diagnostics, surface NaN on /target/selected.
            "distance_unknown_penalty":
                selector_distance_unknown_penalty,
            "reject_if_distance_unknown":
                selector_reject_if_distance_unknown,
        }],
    )
    approach_planner = Node(
        package="go2_semantic_perception",
        executable="approach_goal_planner_node",
        name="approach_goal_planner",
        output="screen",
        parameters=[{
            "selected_topic": "/target/selected",
            "costmap_topic": costmap_topic,
            "goal_pose_topic": "/semantic_goal/goal_pose",
            "candidates_topic": "/semantic_goal/goal_candidates",
            "nav_action_name": nav_action_name,
            "base_frame": base_frame,
            "global_frame": target_frame,
            "num_angle_samples": num_angle_samples,
            "approach_distance_default": approach_distance_default,
            "approach_distance_chair": approach_distance_chair,
            "approach_distance_table": approach_distance_table,
            "approach_distance_desk": approach_distance_desk,
            "approach_distance_box": approach_distance_box,
            "approach_distance_person": approach_distance_person,
            "cost_threshold": cost_threshold,
            "replan_period_sec": replan_period_sec,
            "replan_distance_m": replan_distance_m,
        }],
    )
    coordinator_node = Node(
        package="go2_task_coordinator",
        executable="task_coordinator_node",
        name="task_coordinator",
        output="screen",
        parameters=[{
            "global_frame": target_frame,
            "base_frame": base_frame,
            # Empty: prevent fallback EXPLORE on launch — phase A owns
            # all autonomous exploration.
            "default_target_class": "",
            "get_frontiers_service": "/get_frontiers",
            "nav_action_name": nav_action_name,
            "tick_period_sec": coord_tick_period_sec,
            "log_period_sec": coord_log_period_sec,
            # NL reliability: internal coordinator fallback + nl_parser publish
            # latched feedback. 0.5s gives nl_parser time before tc-fallback.
            "parse_command_fallback_sec": 0.5,
        }],
    )
    nl_parser = Node(
        package="go2_nl_parser",
        executable="nl_parser_node",
        name="nl_parser",
        output="screen",
        parameters=[{
            "input_topic": "/user_command",
            "task_topic": "/semantic_task/request",
            "feedback_topic": "/nl_parser/feedback",
            "global_frame": target_frame,
            "known_classes": nl_known_classes,
            "min_match_confidence": nl_min_match_confidence,
        }],
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        target_frame_arg, base_frame_arg,
        # Day 5
        model_path_arg, classes_arg, conf_arg, iou_arg, device_arg, half_arg,
        publish_overlay_arg,
        # Day 6
        depth_image_arg, camera_info_arg, sync_slop_arg,
        tf_timeout_arg, tf_fallback_latest_arg, min_valid_pixels_arg,
        use_masks_arg, masks_topic_arg, bbox_shrink_arg, depth_percentile_arg,
        # Day 8++ strict at-stamp TF args (Tasks 1 + 2)
        use_stamp_tf_arg, allow_latest_tf_fallback_arg,
        tf_lookup_timeout_arg, keyframe_cache_age_arg,
        max_det_depth_dt_arg,
        # Semantic memory (Day 6 + Day 8+ island association / persistent
        # confirmed landmarks). Without these in the LaunchDescription
        # the corresponding LaunchConfiguration("...") below would
        # raise "launch configuration X does not exist" at parameter-
        # substitution time and abort day8_two_phase.
        nms_radius_arg, position_alpha_arg, confidence_step_arg,
        confidence_decay_arg, min_det_conf_arg, visibility_timeout_arg,
        permanent_after_obs_arg, entity_merge_radius_arg,
        use_island_assoc_arg, island_search_radius_arg,
        island_promo_conf_arg, island_promo_count_arg,
        confirmed_min_conf_arg, keep_confirmed_arg,
        reject_person_near_wall_arg,
        person_wall_clear_arg, table_wall_clear_arg,
        person_max_len_arg, table_max_len_arg,
        invalid_thresh_arg, allow_invalidation_arg,
        # Day 8++ duplicate / promotion / island-required (Tasks 3-5)
        merge_person_radius_arg, merge_table_radius_arg,
        max_confirmed_person_arg, max_confirmed_table_arg,
        person_min_obs_arg, table_min_obs_arg,
        single_obs_island_classes_arg, require_island_classes_arg,
        # Day 8++++ retroactive island enforcement (Task 3)
        mark_unanchored_invalid_arg,
        # Day 8++++ bbox fallback + mask grace + debug stats (Tasks 1-2)
        mask_wait_grace_arg, masks_match_dt_arg, masks_buf_max_arg,
        bbox_fallback_enabled_arg, bbox_fallback_window_arg,
        bbox_fallback_lower_classes_arg, bbox_fallback_conf_scale_arg,
        debug_stats_topic_arg, debug_stats_period_arg,
        # Day 9 — PointCloud cluster anchor + frontier keep-out
        use_pc_anchor_arg, pc_topic_arg, pc_default_radius_arg,
        pc_person_radius_arg, pc_table_radius_arg,
        pc_min_pts_arg, pc_max_pts_arg, pc_tol_arg,
        pc_person_zmin_arg, pc_person_zmax_arg,
        pc_table_zmin_arg, pc_table_zmax_arg,
        pc_buffer_arg, pc_max_per_cloud_arg,
        anchor_stats_topic_arg, anchor_stats_period_arg,
        frontier_reject_inside_islands_arg,
        frontier_semantic_clearance_arg,
        frontier_reject_unknown_pockets_arg,
        obstacle_island_inflation_arg,
        # Day 7 selector + planner
        selector_min_conf_arg, selector_period_arg,
        selector_dist_unknown_pen_arg, selector_reject_dist_unknown_arg,
        costmap_topic_arg, nav_action_arg, num_samples_arg,
        approach_dist_default_arg, approach_dist_chair_arg,
        approach_dist_table_arg, approach_dist_desk_arg,
        approach_dist_box_arg, approach_dist_person_arg,
        cost_threshold_arg, replan_period_arg, replan_distance_arg,
        # Frontier explorer
        map_topic_arg, min_cluster_size_arg, info_gain_radius_arg,
        distance_weight_arg, max_frontiers_arg,
        safety_radius_arg, snap_search_radius_arg, cm_safe_max_cost_arg,
        bbox_xmin_arg, bbox_ymin_arg, bbox_xmax_arg, bbox_ymax_arg,
        # Phase A driver
        map_done_confirm_arg, map_max_aborts_arg, abort_cooldown_arg,
        # Coordinator
        coord_log_period_arg, coord_tick_period_arg,
        # NL parser
        nl_known_classes_arg, nl_min_conf_arg,

        LogInfo(msg=[
            "[day8_two_phase] Phase A: mapping_explorer drives Go2 to "
            "every frontier; semantic_memory writes entities. ",
            "Phase B: publish a string on /user_command "
            "(e.g. 'go to chair') — nl_parser turns it into a "
            "SemanticTask, task_coordinator drives Nav2 to it.",
        ]),
        LogInfo(msg=[
            "[day8_two_phase] target_frame=", target_frame,
            " nl_known_classes=", nl_known_classes,
            " mapping done_confirm=", map_done_confirm_sec,
            " max_aborts=", map_max_aborts,
            " abort_cooldown=", abort_cooldown_sec, "s",
        ]),

        yoloe_node, depth_projector, semantic_memory,
        frontier_node, mapping_explorer,
        target_selector, approach_planner, coordinator_node, nl_parser,
    ])
