"""tf_and_scan.launch.py — minimum sensor TF tree + LaserScan bridge.

Brings up just what slam_toolbox / Nav2 need to start when you
do NOT want the legacy chair_perception YOLO+localizer pipeline:

  /tf_static  base_link -> camera_link
              base_link -> camera_color_optical_frame  (REP-103)
              base_link -> camera_depth_optical_frame  (REP-103)
              base_link -> imu_link
              base_link -> lidar_link
              world      -> map  (visual convenience)
  /scan       sensor_msgs/LaserScan, derived from /lidar/points
              via pointcloud_to_laserscan
  /robot_description
              std_msgs/String holding go2_minimal.urdf, published
              by robot_state_publisher with TRANSIENT_LOCAL durability.
              Lets RViz's "RobotModel" display draw a Go2 silhouette
              at base_link's pose — equivalent to TurtleBot3's
              waffle_pi visual stand-in. To disable, pass
              `with_robot_model:=false` at launch time.

This is a strict subset of chair_perception.launch.py — same TF
quaternions, same pointcloud_to_laserscan params — minus
perception_node and object_localizer_3d_node (Phase-2 chair-only
legacy that is superseded by go2_perception/yoloe_detector_node
and go2_semantic_perception/depth_projector_node in Day 5/6+).

Use this launch in the Day 7+ stack:

    Terminal 1: bash scripts/run_warehouse_ros2.sh
    Terminal 2: ros2 launch go2_bringup_sim tf_and_scan.launch.py     # <-- this
    Terminal 3: ros2 launch go2_bringup_sim nav2.launch.py slam:=True
    Terminal 4: ros2 launch go2_bringup_sim day8.launch.py target_class:=chair
    Terminal 5: bash scripts/run_rviz.sh

Without it: pointcloud_to_laserscan never runs, /scan is empty,
slam_toolbox produces no /map, the `map` frame never appears in
the TF tree, and the entire Day 7/8 stack downstream silently
fails (depth_projector LookupException, Nav2 global_costmap
times out activating, approach_goal_planner skips every plan,
task_coordinator FSM stalls in PLAN_APPROACH_GOAL).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("go2_bringup_sim")
    default_urdf = os.path.join(pkg_share, "urdf", "go2_minimal.urdf")

    with_robot_model_arg = DeclareLaunchArgument(
        "with_robot_model",
        default_value="true",
        description="If true, spawn robot_state_publisher with the Go2 "
                    "URDF so RViz's RobotModel display has something to "
                    "render. Set false to skip (saves ~10 MB RAM).",
    )
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=default_urdf,
        description="Absolute path to the URDF the robot_state_publisher "
                    "should load. Override this if you swap to the full "
                    "go2_description mesh package.",
    )
    with_robot_model = LaunchConfiguration("with_robot_model")
    urdf_path = LaunchConfiguration("urdf_path")
    # ------------------------------------------------------------------
    # Static TF tree under base_link (mirrors chair_perception.launch.py)
    # ------------------------------------------------------------------
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_camera_link_static_tf",
        arguments=[
            "--x", "0.30", "--y", "0.00", "--z", "0.12",
            "--qx", "0.5", "--qy", "-0.5",
            "--qz", "-0.5", "--qw", "0.5",
            "--frame-id", "base_link",
            "--child-frame-id", "camera_link",
        ],
        output="screen",
    )

    # REP-103 optical frame quaternion: see chair_perception.launch.py
    # for the full derivation. (qx, qy, qz, qw) = (0.5, -0.5, 0.5, -0.5)
    # rotates base_link's (X fwd, Y left, Z up) into optical's (X right,
    # Y down, Z forward). Anchored directly under base_link (NOT under
    # camera_link, which uses USD/OpenGL convention -Z forward and would
    # double-rotate).
    _OPTICAL_FRAME_ARGS = [
        "--x", "0.30", "--y", "0.00", "--z", "0.12",
        "--qx", "0.5", "--qy", "-0.5",
        "--qz", "0.5", "--qw", "-0.5",
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
    camera_depth_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_camera_depth_optical_static_tf",
        arguments=_OPTICAL_FRAME_ARGS + [
            "--child-frame-id", "camera_depth_optical_frame",
        ],
        output="screen",
    )

    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_imu_link_static_tf",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0",
            "--frame-id", "base_link",
            "--child-frame-id", "imu_link",
        ],
        output="screen",
    )

    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar_link_static_tf",
        arguments=[
            "--x", "0.10", "--y", "0.00", "--z", "0.20",
            "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0",
            "--frame-id", "base_link",
            "--child-frame-id", "lidar_link",
        ],
        output="screen",
    )

    # ------------------------------------------------------------------
    # world -> map static TF (purely a visual convenience for RViz)
    # ------------------------------------------------------------------
    # slam_toolbox plants `map`'s origin at base_link's first-frame pose
    # (i.e. the Go2 spawn at world (-4, -4) per warehouse_scene.GO2_SPAWN_XYZ).
    # That makes RViz with Fixed Frame=map look like the warehouse is
    # off-centre. Adding a `world` frame whose origin is the warehouse
    # geometric centre (and which `map` lives at -GO2_SPAWN_XYZ from)
    # lets RViz Fixed Frame=world draw a grid centred on the warehouse,
    # while Nav2/SLAM keep using `map` internally — no behaviour change.
    #
    # Math: if a point is at p_map=(0,0) in map frame, its world-frame
    # coords are p_world = p_map + (map's pose in world) = (-4, -4).
    # So translation of (world -> map) static TF = (-4, -4, 0).
    #
    # Keep this in sync with warehouse_scene.GO2_SPAWN_XYZ if you ever
    # move the spawn pose.
    world_to_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_static_tf",
        arguments=[
            "--x", "-4.0", "--y", "-4.0", "--z", "0.0",
            "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0",
            "--frame-id", "world",
            "--child-frame-id", "map",
        ],
        output="screen",
    )

    # ------------------------------------------------------------------
    # /lidar/points (3D PointCloud2) -> /scan (2D LaserScan)
    # ------------------------------------------------------------------
    # Same params as chair_perception.launch.py — kept in sync so a
    # SLAM tune in one place applies to the other. -0.30..+0.50 m
    # vertical slice in lidar_link catches chair legs and tabletops
    # while dropping ceiling/floor returns.
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="lidar_pointcloud_to_laserscan",
        output="screen",
        parameters=[{
            "target_frame": "lidar_link",
            "transform_tolerance": 0.5,
            "min_height": -0.30,
            "max_height": 0.50,
            "angle_min": -3.14159265,
            "angle_max":  3.14159265,
            "angle_increment": 0.00614,    # ~OS1 1024-col resolution
            "scan_time": 0.10,             # 10 Hz LiDAR
            "range_min": 0.30,
            "range_max": 30.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
            "concurrency_level": 1,
        }],
        remappings=[
            ("cloud_in", "/lidar/points"),
            ("scan", "/scan"),
        ],
    )

    # ------------------------------------------------------------------
    # robot_state_publisher: serves go2_minimal.urdf on
    # /robot_description for RViz's RobotModel display.
    # ------------------------------------------------------------------
    # We use Command(["cat", urdf_path]) at launch-evaluation time to
    # read the URDF text; ParameterValue(..., value_type=str) tells
    # rclcpp to keep it as a string (auto-detection would mis-cast a
    # multi-line XML to a list).
    #
    # IMPORTANT: our URDF defines ONLY base_link. All other frames
    # (camera_link, lidar_link, etc.) are still published by the
    # static_transform_publisher nodes above. If you ever expand the
    # URDF to include those links AS URDF children of base_link via
    # joints, REMOVE the corresponding static_transform_publisher to
    # avoid duplicate TF publishers (RViz throws "TF_REPEATED_DATA"
    # on every duplicate).
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["cat ", urdf_path]),
                value_type=str,
            ),
            # Without this RViz reports the model as static at t=0
            # forever and the box "lags" base_link by the slam
            # latency. Match the rest of the stack.
            "use_sim_time": True,
            # 30 Hz publishing rate of the empty /joint_states is fine;
            # we have no joints so there's nothing to broadcast — but
            # robot_state_publisher needs the timer alive for its
            # periodic re-publish of /robot_description.
            "publish_frequency": 30.0,
        }],
        condition=IfCondition(with_robot_model),
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        with_robot_model_arg, urdf_path_arg,
        camera_tf,
        camera_color_optical_tf,
        camera_depth_optical_tf,
        imu_tf,
        lidar_tf,
        world_to_map_tf,
        pointcloud_to_laserscan,
        robot_state_publisher,
    ])
