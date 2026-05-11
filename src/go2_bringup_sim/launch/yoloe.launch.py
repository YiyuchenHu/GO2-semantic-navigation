"""Day 5 — YOLOE open-vocabulary detection launch.

What this launch starts:

  * yoloe_detector_node (go2_perception package)
      Subscribes /camera/color/image_raw, publishes
      /detections (vision_msgs/Detection2DArray) and
      /detections/image (overlay).

What this launch does NOT start:

  * The sim itself (`bash scripts/run_warehouse_ros2.sh` separately).
  * chair_perception.launch.py — the legacy chair-only YOLOv11
    pipeline. The two perception stacks publish on disjoint topic
    namespaces (`/perception/...` vs `/detections`) so they can
    coexist; you choose which one downstream consumers (Day 6
    reprojection, Day 10 command parser) bind to.
  * Static TFs for the camera frames. Those live in
    chair_perception.launch.py and are required for Day 6+
    reprojection — Day 5 only needs the RGB image stream and is
    therefore launchable on top of any node that publishes it.

Why a separate launch (instead of extending chair_perception.launch.py):

The legacy chair_perception.launch.py is a "Phase 1" assembly that
also brings up pointcloud_to_laserscan + the chair-only perception
node + the 3D localizer. Day 5's open-vocabulary detector is
architecturally orthogonal: different model, different message
schema, different set of downstream consumers. Mixing them into one
launch would force every Day 5 dev to either fork the file or carry
both backends in memory. Keep them separate; compose at the
top-level launch (Day 7+) when the system needs both at once.

Parameters
----------
model_path:
    Path or filename of the YOLOE weights. Defaults to
    ``yoloe-11s-seg.pt`` — the 25M-param segmentation variant,
    fast on RTX 4060 / 4070 / 4070Ti class GPUs.
classes:
    Initial text-prompt list. Pass a Python literal:
    ``classes:="['chair','sofa','stool']"``. Day 10's command
    parser will set these dynamically via parameter calls; the
    launch arg is just the bring-up default.
conf_threshold / iou_threshold:
    YOLOE's per-detection confidence cutoff and NMS IoU. Lower
    confidence → higher recall but more false positives. The
    project default 0.4 is a good MVP starting point; drop to
    0.25 if Isaac Sim's chair USDs render as low-poly placeholders
    (training data drift; see "pitfalls" in
    docs/day5_yoloe_status.md).
device / half:
    Torch device selector and FP16 toggle. ``cuda:0`` + half=False
    for first bring-up (most stable); ``cuda:0`` + half=True
    halves GPU memory after you confirm the FP32 path is correct.
publish_overlay:
    Set False on a deployment robot to skip the cv_bridge encode
    + image republish — saves a few ms per frame and ~10 MB/s of
    DDS bandwidth on RGB streams.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="yoloe-11s-seg.pt",
        description="YOLOE weights path (filename auto-resolves under "
                    "Ultralytics' cache; absolute path also fine).",
    )
    # `classes` is a list parameter — declare it WITHOUT a list
    # default at this level (ros2 launch's DeclareLaunchArgument
    # serialises defaults as strings; we hand the str list through
    # to the node's `classes` parameter via PythonEval below). The
    # node itself stores a sensible chair-shaped default so this can
    # be left unset for the MVP run.
    classes_arg = DeclareLaunchArgument(
        "classes",
        default_value="['chair','office chair','stool','folding chair','armchair']",
        description="Python-literal list of YOLOE text prompts. "
                    "Example: classes:=\"['box','crate','pallet']\".",
    )
    conf_arg = DeclareLaunchArgument(
        "conf_threshold",
        default_value="0.4",
        description="Per-detection confidence cutoff (0..1).",
    )
    iou_arg = DeclareLaunchArgument(
        "iou_threshold",
        default_value="0.5",
        description="NMS IoU threshold (0..1).",
    )
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",
        description="Torch device. Falls back to CPU on CUDA failure.",
    )
    half_arg = DeclareLaunchArgument(
        "half",
        default_value="False",
        description="Run YOLOE in FP16 (CUDA only). Halves GPU memory.",
    )
    input_topic_arg = DeclareLaunchArgument(
        "input_topic",
        default_value="/camera/color/image_raw",
        description="RGB image topic to subscribe to.",
    )
    publish_overlay_arg = DeclareLaunchArgument(
        "publish_overlay",
        default_value="True",
        description="If True, publish /detections/image with bbox+mask "
                    "overlay for RViz visualisation.",
    )
    log_period_arg = DeclareLaunchArgument(
        "log_period_sec",
        default_value="5.0",
        description="FPS heartbeat period (s). 0 disables.",
    )

    model_path = LaunchConfiguration("model_path")
    classes = LaunchConfiguration("classes")
    conf = LaunchConfiguration("conf_threshold")
    iou = LaunchConfiguration("iou_threshold")
    device = LaunchConfiguration("device")
    half = LaunchConfiguration("half")
    input_topic = LaunchConfiguration("input_topic")
    publish_overlay = LaunchConfiguration("publish_overlay")
    log_period = LaunchConfiguration("log_period_sec")

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
            "input_topic": input_topic,
            "publish_overlay": publish_overlay,
            "log_period_sec": log_period,
        }],
    )

    return LaunchDescription([
        # use_sim_time on top-level so the YOLOE node's heartbeat
        # log and any timer it adds in the future read sim_time
        # (Isaac Sim publishes /clock; mismatched clocks would make
        # the FPS log oscillate around the right value but with
        # incorrect wall-clock in the timestamps).
        SetParameter(name="use_sim_time", value=True),
        model_path_arg,
        classes_arg,
        conf_arg,
        iou_arg,
        device_arg,
        half_arg,
        input_topic_arg,
        publish_overlay_arg,
        log_period_arg,
        LogInfo(msg=["[yoloe.launch] model_path=", model_path]),
        LogInfo(msg=["[yoloe.launch] classes=", classes]),
        LogInfo(msg=["[yoloe.launch] device=", device, " half=", half]),
        LogInfo(msg=["[yoloe.launch] input_topic=", input_topic]),
        LogInfo(msg=["[yoloe.launch] publish_overlay=", publish_overlay]),
        yoloe_node,
    ])
