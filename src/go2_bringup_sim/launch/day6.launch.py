"""Day 6 — depth reprojection + semantic memory aggregator.

What this launch starts
-----------------------
1. yoloe_detector_node          (go2_perception, Day 5)
   Subscribes /camera/color/image_raw, publishes
   vision_msgs/Detection2DArray on /detections and (default)
   go2_msgs/InstanceMaskArray on /detections/masks for depth sampling.
2. depth_projector_node         (go2_semantic_perception, Day 6)
   Sync /detections + /camera/depth/image_rect_raw +
   /camera/color/camera_info (+ /detections/masks when use_masks:=true),
   reproject each Detection2D into
   `target_frame` (default "map"), publish
   vision_msgs/Detection3DArray on /detections_3d.
3. semantic_memory_aggregator_node (go2_semantic_perception, Day 6)
   Aggregate /detections_3d into go2_msgs/SemanticEntityArray on
   /semantic_map/objects with spatial NMS + confidence decay.
   Publishes /semantic_map/markers for RViz visualisation.

What this launch does NOT start
-------------------------------
* The sim itself (`bash scripts/run_warehouse_ros2.sh` separately).
* chair_perception.launch.py — the LEGACY Phase 1 chair-only
  pipeline. We need its static TFs (camera_link / camera optical
  / lidar_link), so launch it ALONGSIDE this one. The legacy
  perception_node + object_localizer_3d_node are deprecated and
  expected to crash on import (numpy ABI; fixed by
  `bash scripts/install_ml_deps.sh`); their crash does not
  affect the Day 6 stack since we route detections through Day 5
  topic namespaces only.
* Nav2. Day 6 is perception + memory only. Nav2 launches separately
  and consumes /semantic_map/objects from Day 7 onwards.

Why a top-level Day 6 launch instead of nesting yoloe.launch.py:

The two new nodes need to share parameters with yoloe (same
target_frame, same image topic name, etc.) and the operator
benefits from a single console output for the three nodes when
debugging projection / association issues. Nesting via
IncludeLaunchDescription would scatter the logs across three
shells and hide cross-node parameter mismatches.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    # -- YOLOE -------------------------------------------------------------
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="yoloe-11s-seg.pt",
        description="YOLOE weights (Day 5 default).",
    )
    classes_arg = DeclareLaunchArgument(
        "classes",
        default_value=(
            "['chair','office chair','stool','folding chair','armchair',"
            "'table','desk','seat','furniture','box','crate']"
        ),
        description="Initial open-vocab prompt list.",
    )
    conf_arg = DeclareLaunchArgument(
        "conf_threshold", default_value="0.4",
        description="YOLOE per-detection confidence cutoff.",
    )
    iou_arg = DeclareLaunchArgument(
        "iou_threshold", default_value="0.5",
        description="YOLOE NMS IoU threshold.",
    )
    device_arg = DeclareLaunchArgument(
        "device", default_value="cuda:0",
        description="Torch device for YOLOE.",
    )
    half_arg = DeclareLaunchArgument(
        "half", default_value="False",
        description="Run YOLOE in FP16 (CUDA only).",
    )
    publish_overlay_arg = DeclareLaunchArgument(
        "publish_overlay", default_value="True",
        description="Publish /detections/image overlay.",
    )

    # -- Depth projector ---------------------------------------------------
    target_frame_arg = DeclareLaunchArgument(
        "target_frame", default_value="map",
        description="Frame for /detections_3d. Must be "
                    "reachable via tf2 from camera_color_optical_frame.",
    )
    depth_image_arg = DeclareLaunchArgument(
        "depth_image_topic", default_value="/camera/depth/image_rect_raw",
        description="Depth image topic (32FC1 m or 16UC1 mm).",
    )
    camera_info_arg = DeclareLaunchArgument(
        "camera_info_topic", default_value="/camera/color/camera_info",
        description="CameraInfo topic with K matrix.",
    )
    sync_slop_arg = DeclareLaunchArgument(
        "sync_slop", default_value="0.05",
        description="ApproximateTimeSync tolerance (s).",
    )
    tf_timeout_arg = DeclareLaunchArgument(
        "tf_timeout_sec", default_value="1.5",
        description="TF lookup timeout for the reprojection.",
    )
    tf_fallback_latest_arg = DeclareLaunchArgument(
        "tf_fallback_latest_on_time_error",
        default_value="true",
        description="depth_projector: on TF extrapolation, retry with latest "
                    "transform (sim-friendly).",
    )

    # -- Semantic memory aggregator ----------------------------------------
    # Day 6.5: NMS / decay / visibility defaults absorb residual projection
    # jitter; mask-based depth median is primary (see depth_projector_node).
    # See docs/known_issues.md #9 + docs/day7_target_navigation_status.md.
    nms_radius_arg = DeclareLaunchArgument(
        "nms_radius_m", default_value="0.8",
        description="Spatial NMS radius for class-aware merging. "
                    "Day 6.5 raised from 0.3 to 0.8.",
    )
    position_alpha_arg = DeclareLaunchArgument(
        "position_alpha", default_value="0.3",
        description="EMA weight for position update on match.",
    )
    confidence_step_arg = DeclareLaunchArgument(
        "confidence_step_up", default_value="0.15",
        description="Additive confidence bump per matched obs.",
    )
    confidence_decay_arg = DeclareLaunchArgument(
        "confidence_decay_rate", default_value="0.02",
        description="Age-aware exponential confidence decay rate. "
                    "Day 6.5 lowered from 0.05 to 0.02.",
    )
    min_det_conf_arg = DeclareLaunchArgument(
        "min_detection_confidence", default_value="0.4",
        description="Drop detections with score below this before "
                    "feeding the aggregator.",
    )
    min_valid_pixels_arg = DeclareLaunchArgument(
        "min_valid_pixels", default_value="30",
        description="Min finite depth pixels in mask (or bbox fallback ROI) "
                    "before trusting Z (depth_projector).",
    )
    visibility_timeout_arg = DeclareLaunchArgument(
        "visibility_timeout_sec", default_value="5.0",
        description="Mark currently_visible=False after this gap. "
                    "Day 6.5 raised from 2.0 to 5.0 s.",
    )
    permanent_after_obs_arg = DeclareLaunchArgument(
        "permanent_after_observations", default_value="5",
        description="Promote entity to permanent landmark after this "
                    "many observations: confidence stops decaying and "
                    "pruning is skipped. Set 0 to disable.",
    )
    entity_merge_radius_arg = DeclareLaunchArgument(
        "entity_merge_radius_m", default_value="1.5",
        description="Same-class second-pass merge radius (housekeeping). "
                    "Catches duplicate entities from one physical object. "
                    "Set 0 to disable.",
    )

    # -- Day 6.5 — masks + depth_projector -------------------------------
    use_masks_arg = DeclareLaunchArgument(
        "use_masks", default_value="True",
        description="depth_projector: sync InstanceMaskArray from YOLOE "
                    "and use median depth inside mask.",
    )
    masks_topic_arg = DeclareLaunchArgument(
        "masks_topic", default_value="/detections/masks",
        description="go2_msgs/InstanceMaskArray; must match yoloe "
                    "masks_topic.",
    )
    bbox_shrink_arg = DeclareLaunchArgument(
        "bbox_shrink", default_value="0.20",
        description="(Deprecated, mask-less fallback only.) Inset "
                    "fraction before bbox depth ROI.",
    )
    depth_percentile_arg = DeclareLaunchArgument(
        "depth_percentile", default_value="30.0",
        description="(Deprecated, mask-less fallback only.) Percentile "
                    "for bbox ROI depth.",
    )

    # ----------------------------------------------------------------------
    # Substitutions
    # ----------------------------------------------------------------------
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
    nms_radius_m = LaunchConfiguration("nms_radius_m")
    position_alpha = LaunchConfiguration("position_alpha")
    confidence_step_up = LaunchConfiguration("confidence_step_up")
    confidence_decay_rate = LaunchConfiguration("confidence_decay_rate")
    min_detection_confidence = LaunchConfiguration("min_detection_confidence")
    min_valid_pixels = LaunchConfiguration("min_valid_pixels")
    visibility_timeout_sec = LaunchConfiguration("visibility_timeout_sec")
    permanent_after_observations = LaunchConfiguration(
        "permanent_after_observations"
    )
    entity_merge_radius_m = LaunchConfiguration("entity_merge_radius_m")
    bbox_shrink = LaunchConfiguration("bbox_shrink")
    depth_percentile = LaunchConfiguration("depth_percentile")
    use_masks = LaunchConfiguration("use_masks")
    masks_topic = LaunchConfiguration("masks_topic")

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

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        model_path_arg, classes_arg, conf_arg, iou_arg, device_arg, half_arg,
        publish_overlay_arg, target_frame_arg, depth_image_arg,
        camera_info_arg, sync_slop_arg, tf_timeout_arg,
        tf_fallback_latest_arg, nms_radius_arg,
        position_alpha_arg, confidence_step_arg, confidence_decay_arg,
        min_det_conf_arg, min_valid_pixels_arg, visibility_timeout_arg,
        permanent_after_obs_arg, entity_merge_radius_arg,
        # Day 6.5 — masks + depth projector tuning
        use_masks_arg, masks_topic_arg,
        bbox_shrink_arg, depth_percentile_arg,
        LogInfo(msg=["[day6.launch] target_frame=", target_frame]),
        LogInfo(msg=["[day6.launch] yoloe classes=", classes]),
        LogInfo(msg=["[day6.launch] sync_slop=", sync_slop,
                     " tf_timeout_sec=", tf_timeout_sec]),
        LogInfo(msg=["[day6.launch] nms_radius=", nms_radius_m,
                     " decay_rate=", confidence_decay_rate,
                     " min_det_conf=", min_detection_confidence]),
        LogInfo(msg=["[day6.launch] merge_radius=", entity_merge_radius_m,
                     " permanent_after=", permanent_after_observations]),
        LogInfo(msg=["[day6.launch] min_valid_pixels=", min_valid_pixels]),
        yoloe_node,
        depth_projector,
        semantic_memory,
    ])
