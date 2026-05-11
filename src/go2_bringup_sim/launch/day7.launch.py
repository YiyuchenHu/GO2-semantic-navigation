"""Day 7 — semantic target navigation.

Stack
-----
1.  yoloe_detector_node                (go2_perception, Day 5)
2.  depth_projector_node               (go2_semantic_perception, Day 6)
3.  semantic_memory_aggregator_node    (go2_semantic_perception, Day 6)
4.  target_selector_node               (go2_semantic_perception, Day 7)
5.  approach_goal_planner_node         (go2_semantic_perception, Day 7)

Topic chain:

    /camera/color/image_raw  ──►  yoloe ──►  /detections (2D)
                                              │
    /camera/depth/image_rect_raw  ──┐         │
    /camera/color/camera_info  ─────┼─►  depth_projector ──►  /detections_3d
                                              │
                                              ▼
                                semantic_memory  ──►  /semantic_map/objects
                                              │
                                              ▼
                                target_selector  ──►  /target/selected
                                              │
    /global_costmap/costmap (Nav2)  ─────┐    ▼
                                  approach_goal_planner
                                              │
                                              ├─► /semantic_goal/goal_pose (debug)
                                              ├─► /semantic_goal/goal_candidates (RViz)
                                              └─► /navigate_to_pose action ──► Nav2

What this launch does NOT start
-------------------------------
* The simulator (run scripts/run_warehouse_ros2.sh separately).
* nav2.launch.py — Nav2 must be up BEFORE Day 7 sends actions,
  otherwise the action server `/navigate_to_pose` is missing and
  the planner logs a "server not available" warning every replan
  tick. Run:
      ros2 launch go2_bringup_sim nav2.launch.py
  in a separate shell first, wait for `Managed nodes are active`.
* chair_perception.launch.py provides static TFs (camera frames,
  lidar). Launch it alongside Day 7. The legacy perception_node
  is deprecated and will crash on import (numpy ABI; fixed by
  scripts/install_ml_deps.sh) but the crash is harmless to Day 7.

Tunable args reach all five nodes through one LaunchConfiguration
each, so the operator can do:

    ros2 launch go2_bringup_sim day7.launch.py \\
        target_class:=table \\
        approach_distance_default:=1.1 \\
        cost_threshold:=70

without editing code.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # YOLOE (Day 5) args — match day6.launch.py defaults
    # ------------------------------------------------------------------
    model_path_arg = DeclareLaunchArgument(
        "model_path", default_value="yoloe-11s-seg.pt"
    )
    classes_arg = DeclareLaunchArgument(
        "classes",
        default_value=(
            "['chair','office chair','stool','folding chair','armchair',"
            "'table','desk','seat','furniture','box','crate']"
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
    target_frame_arg = DeclareLaunchArgument(
        "target_frame", default_value="map",
        description="Frame for /detections_3d, /semantic_map/objects, "
                    "and the /navigate_to_pose goal pose.",
    )
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
    tf_fallback_latest_arg = DeclareLaunchArgument(
        "tf_fallback_latest_on_time_error",
        default_value="true",
        description="depth_projector: on TF extrapolation at detection stamp, "
                    "retry with latest transform (recommended for Isaac sim).",
    )
    min_valid_pixels_arg = DeclareLaunchArgument(
        "min_valid_pixels", default_value="30"
    )

    # ------------------------------------------------------------------
    # Semantic memory (Day 6) args
    # ------------------------------------------------------------------
    # Day 6.5 NOTE: nms_radius_m, confidence_decay_rate, and
    # visibility_timeout_sec defaults raised below to keep the
    # entity registered through Go2's traverse to the goal pose.
    # Original (more aggressive) values: 0.3 / 0.05 / 2.0 — those
    # work fine for stationary-Go2 demos but kill the entity
    # registry mid-traverse (see docs/known_issues.md #9). Pair
    # mask-based depth (median in depth_projector) with these
    # looser memory knobs; tighter values become viable again if
    # projection jitter drops further.
    nms_radius_arg = DeclareLaunchArgument(
        "nms_radius_m", default_value="0.8",
        description="Spatial NMS radius for class-aware merging. "
                    "Day 6.5 raised from 0.3 to 0.8 to absorb "
                    "frame-to-frame projection jitter while the "
                    "underlying mask-edge bleed is being closed out.",
    )
    position_alpha_arg = DeclareLaunchArgument(
        "position_alpha", default_value="0.3"
    )
    confidence_step_arg = DeclareLaunchArgument(
        "confidence_step_up", default_value="0.15"
    )
    confidence_decay_arg = DeclareLaunchArgument(
        "confidence_decay_rate", default_value="0.02",
        description="Age-aware exponential confidence decay rate. "
                    "Day 6.5 lowered from 0.05 to 0.02 (~50 s "
                    "half-life vs ~14 s) so entities survive Go2 "
                    "turning briefly off-axis during a traverse.",
    )
    min_det_conf_arg = DeclareLaunchArgument(
        "min_detection_confidence", default_value="0.4"
    )
    visibility_timeout_arg = DeclareLaunchArgument(
        "visibility_timeout_sec", default_value="5.0",
        description="Mark entity currently_visible=False after this "
                    "long without a fresh observation. Day 6.5 raised "
                    "from 2 s to 5 s to let entities ride out a "
                    "Nav2-induced turn-away gap.",
    )
    permanent_after_obs_arg = DeclareLaunchArgument(
        "permanent_after_observations", default_value="5",
        description="Promote entity to permanent SLAM-map landmark "
                    "after this many observations: confidence stops "
                    "decaying and pruning is skipped. The Day 8+ use "
                    "case wants table/desk coordinates to persist "
                    "across the whole session even when Go2 turns "
                    "away. Set 0 to disable (legacy behaviour).",
    )
    entity_merge_radius_arg = DeclareLaunchArgument(
        "entity_merge_radius_m", default_value="1.5",
        description="Same-class second-pass merge radius (housekeeping "
                    "tick). Independent of nms_radius_m: this catches "
                    "the 'one desk -> desk_001 + desk_002' failure "
                    "that happens when projection jitter leaks past "
                    "the per-frame NMS radius. Use a value larger "
                    "than nms_radius_m. Set 0 to disable.",
    )

    # ------------------------------------------------------------------
    # Day 6.5 — depth_projector tuning args
    # ------------------------------------------------------------------
    use_masks_arg = DeclareLaunchArgument(
        "use_masks", default_value="True",
        description="When True, depth_projector subscribes to "
                    "/detections/masks (InstanceMaskArray from YOLOE) "
                    "and samples depth at mask pixels (median). "
                    "Disable only when launching depth_projector "
                    "without YOLOE masks — the synchroniser would "
                    "otherwise stall waiting for the 4th input.",
    )
    masks_topic_arg = DeclareLaunchArgument(
        "masks_topic", default_value="/detections/masks",
        description="Topic name for go2_msgs/InstanceMaskArray. "
                    "Must match yoloe_detector's masks_topic.",
    )
    # ---- DEPRECATED — used only when `use_masks:=false` -----------
    bbox_shrink_arg = DeclareLaunchArgument(
        "bbox_shrink", default_value="0.20",
        description="(Deprecated, mask-less fallback only.) Inset "
                    "fraction (each side) before sampling the depth "
                    "ROI when no mask is available.",
    )
    depth_percentile_arg = DeclareLaunchArgument(
        "depth_percentile", default_value="30.0",
        description="(Deprecated, mask-less fallback only.) Percentile "
                    "(1..99) used to reduce ROI depth to one Z when "
                    "no mask is available.",
    )

    # ------------------------------------------------------------------
    # Day 7 — target selector args
    # ------------------------------------------------------------------
    target_class_arg = DeclareLaunchArgument(
        "target_class", default_value="chair",
        description="Class label to look for in /semantic_map/objects. "
                    "Day 10 command interface rewrites this via "
                    "ros2 param set.",
    )
    selector_min_conf_arg = DeclareLaunchArgument(
        "selector_min_confidence", default_value="0.20",
        description="Minimum SemanticEntity.confidence to be a "
                    "selector candidate. Day 6.5 lowered from 0.30 "
                    "to 0.20 to keep entities selectable through "
                    "the early phase of a traverse, when an entity "
                    "with confidence_step_up=0.15 has only been "
                    "observed once or twice. Combine with the "
                    "lower confidence_decay_rate so this floor "
                    "isn't hit by a long off-axis turn either.",
    )
    selector_period_arg = DeclareLaunchArgument(
        "selector_period_sec", default_value="0.5",
        description="Selector tick period.",
    )
    base_frame_arg = DeclareLaunchArgument(
        "base_frame", default_value="base_link"
    )

    # ------------------------------------------------------------------
    # Day 7 — approach goal planner args
    # ------------------------------------------------------------------
    costmap_topic_arg = DeclareLaunchArgument(
        "costmap_topic", default_value="/global_costmap/costmap",
        description="Nav2 global costmap — Day 4 default.",
    )
    nav_action_arg = DeclareLaunchArgument(
        "nav_action_name", default_value="/navigate_to_pose",
        description="Nav2 NavigateToPose action server name.",
    )
    num_samples_arg = DeclareLaunchArgument(
        "num_angle_samples", default_value="16",
        description="Ring samples around the target.",
    )
    approach_dist_default_arg = DeclareLaunchArgument(
        "approach_distance_default", default_value="0.9",
        description="Fallback approach stand-off distance (m).",
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
        "cost_threshold", default_value="60",
        description="Reject candidates with costmap value above this.",
    )
    replan_period_arg = DeclareLaunchArgument(
        "replan_period_sec", default_value="1.0"
    )
    replan_distance_arg = DeclareLaunchArgument(
        "replan_distance_m", default_value="0.10"
    )

    # ------------------------------------------------------------------
    # Substitutions
    # ------------------------------------------------------------------
    model_path = LaunchConfiguration("model_path")
    classes = LaunchConfiguration("classes")
    conf = LaunchConfiguration("conf_threshold")
    iou = LaunchConfiguration("iou_threshold")
    device = LaunchConfiguration("device")
    half = LaunchConfiguration("half")
    publish_overlay = LaunchConfiguration("publish_overlay")
    target_frame = LaunchConfiguration("target_frame")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    sync_slop = LaunchConfiguration("sync_slop")
    tf_timeout_sec = LaunchConfiguration("tf_timeout_sec")
    tf_fallback_latest_on_time_error = LaunchConfiguration(
        "tf_fallback_latest_on_time_error"
    )
    min_valid_pixels = LaunchConfiguration("min_valid_pixels")
    bbox_shrink = LaunchConfiguration("bbox_shrink")
    depth_percentile = LaunchConfiguration("depth_percentile")
    use_masks = LaunchConfiguration("use_masks")
    masks_topic = LaunchConfiguration("masks_topic")
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
    target_class = LaunchConfiguration("target_class")
    selector_min_confidence = LaunchConfiguration("selector_min_confidence")
    selector_period_sec = LaunchConfiguration("selector_period_sec")
    base_frame = LaunchConfiguration("base_frame")
    costmap_topic = LaunchConfiguration("costmap_topic")
    nav_action_name = LaunchConfiguration("nav_action_name")
    num_angle_samples = LaunchConfiguration("num_angle_samples")
    approach_distance_default = LaunchConfiguration("approach_distance_default")
    approach_distance_chair = LaunchConfiguration("approach_distance_chair")
    approach_distance_table = LaunchConfiguration("approach_distance_table")
    approach_distance_desk = LaunchConfiguration("approach_distance_desk")
    approach_distance_box = LaunchConfiguration("approach_distance_box")
    approach_distance_person = LaunchConfiguration("approach_distance_person")
    cost_threshold = LaunchConfiguration("cost_threshold")
    replan_period_sec = LaunchConfiguration("replan_period_sec")
    replan_distance_m = LaunchConfiguration("replan_distance_m")

    # ------------------------------------------------------------------
    # Nodes
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
            "min_valid_pixels": min_valid_pixels,
            "bbox_shrink": bbox_shrink,
            "depth_percentile": depth_percentile,
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
            "frame_id": target_frame,
            "nms_radius_m": nms_radius_m,
            "position_alpha": position_alpha,
            "confidence_step_up": confidence_step_up,
            "confidence_decay_rate": confidence_decay_rate,
            "min_detection_confidence": min_detection_confidence,
            "visibility_timeout_sec": visibility_timeout_sec,
            "permanent_after_observations": permanent_after_observations,
            "entity_merge_radius_m": entity_merge_radius_m,
        }],
    )

    target_selector = Node(
        package="go2_semantic_perception",
        executable="target_selector_node",
        name="target_selector",
        output="screen",
        parameters=[{
            "entities_topic": "/semantic_map/objects",
            "selected_topic": "/target/selected",
            "target_class": target_class,
            "min_confidence": selector_min_confidence,
            "select_period_sec": selector_period_sec,
            "base_frame": base_frame,
            "global_frame": target_frame,
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

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        # Day 5
        model_path_arg, classes_arg, conf_arg, iou_arg, device_arg, half_arg,
        publish_overlay_arg,
        # Day 6
        target_frame_arg, depth_image_arg, camera_info_arg, sync_slop_arg,
        tf_timeout_arg, tf_fallback_latest_arg, min_valid_pixels_arg,
        nms_radius_arg,
        position_alpha_arg, confidence_step_arg, confidence_decay_arg,
        min_det_conf_arg, visibility_timeout_arg,
        permanent_after_obs_arg, entity_merge_radius_arg,
        # Day 6.5 — depth projector tuning + mask-aware sampling
        bbox_shrink_arg, depth_percentile_arg,
        use_masks_arg, masks_topic_arg,
        # Day 7 selector
        target_class_arg, selector_min_conf_arg, selector_period_arg,
        base_frame_arg,
        # Day 7 planner
        costmap_topic_arg, nav_action_arg, num_samples_arg,
        approach_dist_default_arg, approach_dist_chair_arg,
        approach_dist_table_arg, approach_dist_desk_arg,
        approach_dist_box_arg, approach_dist_person_arg,
        cost_threshold_arg, replan_period_arg, replan_distance_arg,
        LogInfo(msg=["[day7.launch] target_frame=", target_frame,
                     " target_class=", target_class]),
        LogInfo(msg=["[day7.launch] depth_projector tf_fallback_latest_on_time_error=",
                     tf_fallback_latest_on_time_error]),
        LogInfo(msg=["[day7.launch] cost_threshold=", cost_threshold,
                     " num_samples=", num_angle_samples,
                     " approach_default=", approach_distance_default]),
        LogInfo(msg=["[day7.launch] nav_action=", nav_action_name,
                     " costmap=", costmap_topic]),
        LogInfo(msg=["[day7.launch] semantic memory: nms_r=", nms_radius_m,
                     " merge_r=", entity_merge_radius_m,
                     " permanent_after=", permanent_after_observations,
                     " decay_rate=", confidence_decay_rate]),
        yoloe_node,
        depth_projector,
        semantic_memory,
        target_selector,
        approach_planner,
    ])
