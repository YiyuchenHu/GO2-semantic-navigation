"""Phase 1 chair-only perception launch.

Starts the minimum subset of nodes needed to produce:
  /perception/detections_2d    (sensor_msgs/Image  -> chair 2D detections)
  /perception/masks            (chair instance masks)
  /perception/objects_3d       (chair 3D observations in base_link + odom)
  /scan                        (sensor_msgs/LaserScan, derived from the
                                3D LiDAR PointCloud2 emitted by the sim)

Prerequisites (Phase 0 must be running):
  * sim/run_go2_warehouse_ros2.py is up (via scripts/run_warehouse_ros2.sh)
  * /camera/color/image_raw, /camera/color/camera_info,
    /camera/depth/image_rect_raw, /imu/data, /lidar/points are flowing
  * /tf publishes odom -> base_link

This launch is also the home of the rigid sensor extrinsics published
on /tf_static, plus the pointcloud_to_laserscan bridge:
  base_link -> camera_link
  base_link -> camera_color_optical_frame   (REP-103 X right, Y down, Z fwd)
  base_link -> camera_depth_optical_frame
  base_link -> imu_link
  base_link -> lidar_link
  /lidar/points (PointCloud2) -> /scan (LaserScan, slam_toolbox-friendly)

Camera prim pose, hard-coded in sim/run_go2_warehouse_ros2.py:
  translation = (0.30, 0.00, 0.12) in base
  orientation = -90deg about +Y so the camera's -Z optical axis points
  along +X of base_link.

LiDAR prim pose, hard-coded in sim/run_go2_warehouse_ros2.py:
  translation = (0.10, 0.00, 0.20) in base, identity orientation.

Explicitly NOT started here (deferred to later phases):
  semantic memory, target selection, planning, navigation execution,
  task coordination, safety monitor.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    target_class_arg = DeclareLaunchArgument(
        "target_class",
        default_value="chair",
        description="Semantic class name used to flag target candidates and to "
                    "filter /perception/detections_2d. Phase 1 default: chair.",
    )
    only_target_arg = DeclareLaunchArgument(
        "only_target",
        default_value="true",
        description="If true, drop non-target detections before publishing.",
    )
    yolo_model_arg = DeclareLaunchArgument(
        "yolo_model",
        default_value="yolo11l-seg.pt",
        description="Ultralytics YOLO segmentation model weights file.",
    )
    global_frame_arg = DeclareLaunchArgument(
        "global_frame",
        default_value="odom",
        description="Global TF frame the 3D localizer projects observations into. "
                    "Phase 1 default 'odom' matches the sim TF tree.",
    )
    # Empirically the simulated warehouse chair gets confidently confused
    # by YOLO for other indoor-furniture COCO classes (observed: bench,
    # couch, sofa, armchair, bed). We normalise all of those to 'chair'
    # rather than tightening the YOLO threshold, which would drop real
    # chair detections too. Override at runtime via
    #   --ros-args -p perception_node.target_class_aliases:=['chair',...]
    chair_aliases = ["chair", "couch", "bench", "sofa", "armchair", "bed"]

    target_class = LaunchConfiguration("target_class")
    only_target = LaunchConfiguration("only_target")
    yolo_model = LaunchConfiguration("yolo_model")
    global_frame = LaunchConfiguration("global_frame")

    # ------------------------------------------------------------------
    # Static TF tree under base_link
    # ------------------------------------------------------------------
    # Phase 0 sim only publishes the dynamic odom -> base_link transform
    # (via PubRawTF). Every sensor frame below base_link is rigidly
    # attached and must therefore be advertised on /tf_static here.
    #
    # Camera extrinsic, hard-coded in sim/run_go2_warehouse_ros2.py:
    #   translation = (0.30, 0.00, 0.12) in base_link
    #   orientation_wxyz = (0.5, 0.5, -0.5, -0.5)
    #     = (-90° about +Y)  followed by  (+90° about +X)
    # The first rotation aims the USD camera's -Z (its native looking
    # direction) along base_link's +X; the second rolls the USD camera
    # +90° around its optical axis so image-up aligns with base_link +Z
    # (world up) instead of base_link +Y (left). Without this second
    # rotation the published RGB image is sideways.
    #
    # In static_transform_publisher's xyzw form: qx=0.5, qy=-0.5,
    # qz=-0.5, qw=0.5. After this transform:
    #   camera_link +X = base_link -Y   (right of robot)
    #   camera_link +Y = base_link +Z   (world up)
    #   camera_link +Z = base_link -X   (back of robot)  — i.e. the
    #     USD/OpenGL "behind the lens" axis
    #
    # `camera_link` therefore follows the USD/OpenGL convention (looking
    # along -Z). REP-103-compliant optical frames are still anchored
    # directly to base_link below with the canonical optical quaternion,
    # independent of camera_link's pose.
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_camera_link_static_tf",
        arguments=[
            "--x", "0.30",
            "--y", "0.00",
            "--z", "0.12",
            "--qx", "0.5",
            "--qy", "-0.5",
            "--qz", "-0.5",
            "--qw", "0.5",
            "--frame-id", "base_link",
            "--child-frame-id", "camera_link",
        ],
        output="screen",
    )

    # REP-103 optical frame quaternion (parent = REP-103 body, child =
    # optical): rotates body's (X forward, Y left, Z up) basis into
    # optical's (X right, Y down, Z forward) basis. The quaternion is
    # the well-known (qx, qy, qz, qw) = (0.5, -0.5, 0.5, 0.5).
    # We anchor the optical frames directly to base_link (rather than
    # camera_link) because camera_link in this codebase is the
    # USD/OpenGL convention, so chaining optical under it would
    # double-rotate. The translation matches the USD camera prim's
    # (0.30, 0.00, 0.12) offset in base_link.
    _OPTICAL_FRAME_ARGS = [
        "--x", "0.30",
        "--y", "0.00",
        "--z", "0.12",
        "--qx", "0.5",
        "--qy", "-0.5",
        "--qz", "0.5",
        "--qw", "0.5",
        "--frame-id", "base_link",
    ]
    camera_color_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_camera_color_optical_static_tf",
        arguments=_OPTICAL_FRAME_ARGS + [
            "--child-frame-id", "camera_color_optical_frame",
        ],
        output="screen",
    )
    # Depth shares the same render product as color (single Camera prim
    # in run_go2_warehouse_ros2.py), so its optical center coincides
    # with the color one. Publish a separate frame so REP-105-aware
    # tools that look up `camera_depth_optical_frame` still resolve.
    camera_depth_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_camera_depth_optical_static_tf",
        arguments=_OPTICAL_FRAME_ARGS + [
            "--child-frame-id", "camera_depth_optical_frame",
        ],
        output="screen",
    )

    # IMU prim: created in run_go2_warehouse_ros2.py at translation
    # (0, 0, 0) with identity orientation under /World/Go2/base. So
    # imu_link == base_link in pose; the static TF is identity but
    # still needed so REP-105-aware nodes that query
    # tf2 lookup("imu_link", ...) succeed.
    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_imu_link_static_tf",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--qx", "0.0",
            "--qy", "0.0",
            "--qz", "0.0",
            "--qw", "1.0",
            "--frame-id", "base_link",
            "--child-frame-id", "imu_link",
        ],
        output="screen",
    )

    # LiDAR prim: created in run_go2_warehouse_ros2.py at translation
    # (0.10, 0.00, 0.20) with identity orientation under
    # /World/Go2/base/lidar. The PointCloud2 topic /lidar/points is
    # tagged with frame_id=lidar_link, so this static TF must match
    # the prim pose for tf2 lookups to be geometrically correct.
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar_link_static_tf",
        arguments=[
            "--x", "0.10",
            "--y", "0.00",
            "--z", "0.20",
            "--qx", "0.0",
            "--qy", "0.0",
            "--qz", "0.0",
            "--qw", "1.0",
            "--frame-id", "base_link",
            "--child-frame-id", "lidar_link",
        ],
        output="screen",
    )

    # /lidar/points (PointCloud2, 3D from Ouster OS1-32 stand-in) ->
    # /scan (sensor_msgs/LaserScan) via pointcloud_to_laserscan.
    # slam_toolbox + Nav2 obstacle layer both consume LaserScan as
    # their primary 2D sensor input. Mirrors the real Go2 pipeline
    # (Livox MID-360 -> PointCloud2 -> pointcloud_to_laserscan ->
    # /scan), so the SLAM/Nav2 configuration that's tuned here will
    # transfer straight onto hardware.
    #
    # Requires the apt package on Jazzy:
    #     sudo apt install ros-jazzy-pointcloud-to-laserscan
    # If the package isn't installed, this Node will print a clear
    # "executable 'pointcloud_to_laserscan_node' not found" error at
    # launch time but won't take down the rest of the launch.
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="lidar_pointcloud_to_laserscan",
        output="screen",
        parameters=[{
            # Project all 3D points within this height band (relative
            # to the target frame) onto the 2D scan plane. -0.30..+0.50
            # gives us a 0.8 m vertical slice centred on the LiDAR's
            # mounting height — wide enough to catch chair legs and
            # tabletops, narrow enough to drop ceiling and floor.
            "target_frame": "lidar_link",
            # transform_tolerance was 0.05 originally — too tight when
            # /lidar/points runs slow (~4 Hz under GPU contention) or
            # when sim_time has small jitter. Bump to 0.5 s so a stale
            # TF lookup window doesn't silently drop every cloud.
            "transform_tolerance": 0.5,
            "min_height": -0.30,
            "max_height": 0.50,
            # Full 360° sweep at ~0.5° resolution (matches OS1's 1024
            # horizontal columns: 2*pi / 1024 ≈ 0.00614 rad ≈ 0.35°).
            "angle_min": -3.14159265,
            "angle_max":  3.14159265,
            "angle_increment": 0.00614,
            "scan_time": 0.10,        # 10 Hz LiDAR
            "range_min": 0.30,        # OS1 minimum reliable range
            "range_max": 30.0,        # well below OS1's 120 m, but enough for the warehouse
            "use_inf": True,
            "inf_epsilon": 1.0,
            "concurrency_level": 1,
        }],
        remappings=[
            ("cloud_in", "/lidar/points"),
            ("scan", "/scan"),
        ],
    )

    perception = Node(
        package="go2_perception",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[{
            "yolo_model": yolo_model,
            "default_target_class": target_class,
            "only_publish_target_class": only_target,
            "target_class_aliases": chair_aliases,
            "log_period_sec": 1.0,
        }],
    )

    localizer = Node(
        package="go2_object_localization",
        executable="object_localizer_3d_node",
        name="object_localizer_3d_node",
        output="screen",
        parameters=[{
            # Phase 0 sim only publishes /camera/color/camera_info.
            "depth_info_topic": "/camera/color/camera_info",
            "global_frame": global_frame,
            "base_frame": "base_link",
            "log_period_sec": 1.0,
        }],
    )

    return LaunchDescription([
        # Sim publishes /clock and stamps every message with sim time
        # (PubClock + IsaacReadSimulationTime in sim/run_go2_warehouse_ros2.py).
        # All ROS-side nodes below MUST consume sim time, otherwise tf2
        # extrapolation fails the moment sim_time != wall_time.
        SetParameter(name="use_sim_time", value=True),
        target_class_arg,
        only_target_arg,
        yolo_model_arg,
        global_frame_arg,
        camera_tf,
        camera_color_optical_tf,
        camera_depth_optical_tf,
        imu_tf,
        lidar_tf,
        pointcloud_to_laserscan,
        perception,
        localizer,
    ])
