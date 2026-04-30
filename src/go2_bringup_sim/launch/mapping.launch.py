"""Day 3 mapping launch — chair perception + slam_toolbox.

Composition:
  1. Includes `chair_perception.launch.py` so we get
       /tf_static for base_link → camera_link / camera_*_optical_frame
                              / imu_link / lidar_link
       /scan via pointcloud_to_laserscan over /lidar/points
       (and the YOLO chair detector pipeline as a free side-effect)
  2. Starts slam_toolbox::async_slam_toolbox_node in mapping mode,
     which subscribes to /scan and publishes:
       /map                 nav_msgs/OccupancyGrid    (1 Hz)
       /tf  (map → odom)    tf2_msgs/TFMessage        (20 Hz)

Result by Day 3 acceptance:
  Driving the Go2 around the warehouse with /cmd_vel produces a
  cleaned-up 2D occupancy grid in `map` frame, which can then be
  saved with `scripts/save_map.sh` and consumed by Day 4's Nav2
  bringup.

Prerequisites (see also docs/phase0_status.md and docs/phase3_status.md):
  * Phase 0 sim is running (`bash scripts/run_warehouse_ros2.sh`)
  * apt packages installed:
      sudo apt install ros-jazzy-slam-toolbox \\
                       ros-jazzy-nav2-map-server \\
                       ros-jazzy-pointcloud-to-laserscan
  * Workspace built: `colcon build --symlink-install --packages-select go2_bringup_sim`
"""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("go2_bringup_sim")

    slam_params_default = join(pkg_share, "config", "slam",
                               "slam_toolbox_mapping.yaml")

    slam_params_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=slam_params_default,
        description="Path to the slam_toolbox parameters YAML. Default is "
                    "the project-tuned config under "
                    "go2_bringup_sim/config/slam/slam_toolbox_mapping.yaml.",
    )
    target_class_arg = DeclareLaunchArgument(
        "target_class",
        default_value="chair",
        description="Forwarded to chair_perception. Phase 1 default: chair.",
    )
    global_frame_arg = DeclareLaunchArgument(
        "global_frame",
        default_value="map",
        description="Global frame for downstream consumers. Once SLAM is "
                    "online, semantic map / Nav2 should run in 'map' rather "
                    "than 'odom' so they benefit from loop closure. Default "
                    "switched to 'map' here (vs 'odom' in chair_*.launch.py "
                    "stack) because slam_toolbox publishes map → odom.",
    )

    slam_params_file = LaunchConfiguration("slam_params_file")
    target_class = LaunchConfiguration("target_class")
    global_frame = LaunchConfiguration("global_frame")

    # ---- Phase 1 perception (provides /tf_static + /scan) ---------
    # We forward global_frame=map so that the localizer / perception
    # nodes attempt to put 3D observations directly in `map`. If
    # slam_toolbox isn't ready yet they will gracefully fall back to
    # `base_link`-only output (see object_localizer_3d_node).
    chair_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_share, "launch", "chair_perception.launch.py"),
        ),
        launch_arguments={
            "target_class": target_class,
            "global_frame": global_frame,
        }.items(),
    )

    # ---- slam_toolbox via its own online_async_launch.py ----------
    #
    # async_slam_toolbox_node is a LIFECYCLE node — it starts in the
    # `unconfigured` state and only subscribes to /scan / /tf and
    # publishes /map + map→odom AFTER going through CONFIGURE +
    # ACTIVATE transitions. Spawning it as a plain `launch_ros.Node`
    # leaves it dormant: parameters from our YAML never load, /scan
    # subscription is never created, /map never publishes, and check_day3
    # fails with "no OccupancyGrid received".
    #
    # The slam_toolbox package ships an `online_async_launch.py` that
    # wraps the node in a `LifecycleNode` + `EmitEvent(ChangeState,
    # CONFIGURE)` + `OnStateTransition(goal_state="inactive") ->
    # EmitEvent(ChangeState, ACTIVATE)`. Including it instead of
    # rolling our own Node gives us autostart for free, and we just
    # forward our tuned slam_params_file.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory("slam_toolbox"),
                 "launch", "online_async_launch.py"),
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription([
        # use_sim_time MUST cascade to slam_toolbox or its scan TFs
        # will pull wall-time stamps and mismatch the sim's sim-time
        # /scan + /tf, breaking ICP from the very first frame.
        SetParameter(name="use_sim_time", value=True),
        slam_params_arg,
        target_class_arg,
        global_frame_arg,
        chair_perception_launch,
        slam_launch,
    ])
