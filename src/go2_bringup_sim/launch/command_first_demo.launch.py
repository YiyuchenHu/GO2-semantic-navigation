"""command_first_demo.launch.py — single-launch command-first online
exploration with social-aware costmap inflation.

Operator workflow
-----------------
1. Terminal 1: bash scripts/run_warehouse_ros2.sh
   (Isaac Sim warehouse — must already be publishing /camera/color/
   image_raw, /lidar/points, /tf for odom→base_link and /clock.)

2. Wait until Isaac Sim is fully up (camera + LiDAR topics flowing).

3. Terminal 2:  ros2 launch go2_bringup_sim command_first_demo.launch.py
   This brings up everything else in one process tree:
     * tf_and_scan.launch.py  (static TFs + pointcloud_to_laserscan)
     * nav2.launch.py slam:=True params_file:=nav2_params_social.yaml
     * YOLOE detector (with `person` in the class allowlist)
     * depth_projector + semantic_memory_aggregator
     * target_selector + approach_goal_planner
     * frontier_explorer + task_coordinator
     * nl_parser
     * social_obstacle_publisher  (publishes /social_obstacles)

4. Wait ~20 s for slam_toolbox to publish first /map.

5. Terminal 3:
     ros2 topic pub --once /user_command std_msgs/msg/String \\
         "data: 'find chair'"

6. Watch:
     ros2 topic echo /task_coordinator/state
     ros2 topic echo /task/status
   Expected progression:
     IDLE → PARSE_COMMAND → CHECK_MEMORY → TARGET_NOT_FOUND
     → EXPLORE → ... → TARGET_FOUND → PLAN_APPROACH_GOAL
     → NAVIGATE_TO_GOAL → ARRIVED

Differences vs day8_two_phase.launch.py
---------------------------------------
* mapping_explorer_node is NOT launched. task_coordinator's EXPLORE
  state owns the autonomous exploration loop directly via
  /get_frontiers; this avoids the Nav2 action-server contention
  documented in the prior FSM audit.
* nav2 starts with nav2_params_social.yaml (inflation_radius=0.8m
  on both costmaps, /social_obstacles registered as a marking-only
  PointCloud2 source on the obstacle layer).
* social_obstacle_publisher_node runs to materialise person
  rings on /social_obstacles at 5 Hz.
* task_coordinator's tf_startup_grace_sec is bumped to 20.0 (vs the
  in-code default 15.0) — extra-conservative for Isaac Sim 5090
  cold boot where /map can lag the first /scan by ~10–15 s.
* The legacy command_parser_node from go2_command_interface is NOT
  launched; nl_parser_node from go2_nl_parser is the sole
  /user_command consumer.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("go2_bringup_sim")
    default_social_params = os.path.join(
        pkg_share, "config", "nav2", "nav2_params_social.yaml"
    )

    # ------------------------------------------------------------------
    # Launch arguments — keep the surface narrow on purpose. Demo
    # operator only needs to override these in unusual situations.
    # ------------------------------------------------------------------
    target_frame_arg = DeclareLaunchArgument(
        "target_frame", default_value="map",
        description="Frame for /detections_3d, /semantic_map/objects, "
                    "/get_frontiers' robot_pose, and Nav2 goals.",
    )
    base_frame_arg = DeclareLaunchArgument(
        "base_frame", default_value="base_link"
    )
    classes_arg = DeclareLaunchArgument(
        "classes",
        # Open-vocabulary class prompts for YOLOE. Adding new
        # target types requires only extending this list — no
        # other code changes.
        # `person` is required for social-cost inflation.
        # `table` / `dining table` enable "find table" commands
        # in addition to the chair variants.
        default_value=(
            "['chair','office chair','stool','folding chair',"
            "'armchair','person','table','dining table']"
        ),
        description="YOLOE open-vocabulary class prompts. Must "
                    "include the navigation target class AND "
                    "'person' for social-cost inflation.",
    )
    conf_arg = DeclareLaunchArgument(
        "conf_threshold", default_value="0.4",
        description="YOLOE per-detection confidence floor. 0.4 is "
                    "the tested sweet-spot for chair + person "
                    "in the warehouse scene.",
    )
    nav2_params_arg = DeclareLaunchArgument(
        "nav2_params_file", default_value=default_social_params,
        description="Path to the social-aware Nav2 params YAML. "
                    "Defaults to nav2_params_social.yaml shipped in "
                    "go2_bringup_sim/config/nav2/.",
    )
    nl_known_classes_arg = DeclareLaunchArgument(
        "nl_known_classes",
        # Mirror YOLOE's allowlist so /user_command tokens always
        # resolve to a class the perception pipeline can detect.
        default_value="['chair','table','person']",
    )
    tf_grace_arg = DeclareLaunchArgument(
        "tf_startup_grace_sec", default_value="20.0",
        description="task_coordinator wall-clock budget for SLAM TF "
                    "and /map readiness on cold boot. 20s is "
                    "extra-conservative for Isaac Sim 5090 where "
                    "the first /map can lag the first /scan by "
                    "10–15s under GPU contention.",
    )

    target_frame = LaunchConfiguration("target_frame")
    base_frame = LaunchConfiguration("base_frame")
    classes = LaunchConfiguration("classes")
    conf_threshold = LaunchConfiguration("conf_threshold")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nl_known_classes = LaunchConfiguration("nl_known_classes")
    tf_startup_grace_sec = LaunchConfiguration("tf_startup_grace_sec")

    # ------------------------------------------------------------------
    # Pre-included launches — TF + Nav2/SLAM
    # ------------------------------------------------------------------
    # tf_and_scan publishes the static TF tree (base_link→camera_*,
    # base_link→lidar_link, world→map) and runs pointcloud_to_laserscan
    # to derive /scan from /lidar/points. Without it slam_toolbox
    # never sees a scan and /map stays empty forever.
    tf_and_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "tf_and_scan.launch.py")
        ),
    )
    # nav2.launch.py with slam:=True spins up async slam_toolbox +
    # the full Nav2 server set, parameterised by the social YAML so
    # the obstacle layer registers /social_obstacles and the
    # inflation layer enforces the 0.8m social personal-space halo.
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "slam": "True",
            "params_file": nav2_params_file,
        }.items(),
    )

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------
    yoloe_node = Node(
        package="go2_perception",
        executable="yoloe_detector_node",
        name="yoloe_detector",
        output="screen",
        parameters=[{
            # All other knobs (model_path, iou_threshold, device,
            # half, input_topic, masks_topic, publish_overlay,
            # log_period_sec) keep their in-code defaults — those
            # match the day8_two_phase tested values.
            "classes": classes,
            "conf_threshold": conf_threshold,
        }],
    )
    depth_projector = Node(
        package="go2_semantic_perception",
        executable="depth_projector_node",
        name="depth_projector",
        output="screen",
        parameters=[{
            "target_frame": target_frame,
        }],
    )
    semantic_memory = Node(
        package="go2_semantic_perception",
        executable="semantic_memory_aggregator_node",
        name="semantic_memory_aggregator",
        output="screen",
        parameters=[{
            "frame_id": target_frame,
        }],
    )

    # ------------------------------------------------------------------
    # Frontier provider (the EXPLORE state's /get_frontiers backend)
    # ------------------------------------------------------------------
    # NOTE: mapping_explorer_node is INTENTIONALLY NOT LAUNCHED
    # here. task_coordinator owns the autonomous exploration loop
    # via its own EXPLORE state, which calls /get_frontiers and
    # drives Nav2 directly. Running mapping_explorer in parallel
    # would have two clients competing for /navigate_to_pose
    # (Blocker #2 from the prior audit).
    frontier_node = Node(
        package="go2_navigation",
        executable="frontier_explorer_node",
        name="frontier_explorer",
        output="screen",
        parameters=[{
            # Warehouse extends to ~±7m; default ±5.5m bbox was
            # filtering out all frontier candidates and causing
            # EXPLORE to FAIL immediately.
            # Set to ±8.0m to cover full warehouse extent (~±7m)
            # with a 1m margin for wall-edge phantom frontiers.
            "bbox_xmin": -8.0,
            "bbox_ymin": -8.0,
            "bbox_xmax":  8.0,
            "bbox_ymax":  8.0,
        }],
    )

    # ------------------------------------------------------------------
    # Semantic target loop (selector + approach planner + coordinator
    # + NL parser)
    # ------------------------------------------------------------------
    target_selector = Node(
        package="go2_semantic_perception",
        executable="target_selector_node",
        name="target_selector",
        output="screen",
        parameters=[{
            # Empty target_class — task_coordinator pushes the
            # active class via AsyncParameterClient as soon as a
            # SemanticTask arrives. See task_coordinator's
            # _set_target_selector_class().
            "target_class": "",
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
            "base_frame": base_frame,
            "global_frame": target_frame,
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
            # Empty default_target_class — the only way into EXPLORE
            # is via a real /user_command + nl_parser SemanticTask.
            "default_target_class": "",
            # Patch A — same wall-clock grace knob now governs both
            # TF and /map readiness. 20s for cold-boot Isaac Sim.
            "tf_startup_grace_sec": tf_startup_grace_sec,
        }],
    )
    nl_parser = Node(
        package="go2_nl_parser",
        executable="nl_parser_node",
        name="nl_parser",
        output="screen",
        parameters=[{
            "global_frame": target_frame,
            "known_classes": nl_known_classes,
        }],
    )

    # ------------------------------------------------------------------
    # Social-aware costmap source
    # ------------------------------------------------------------------
    social_obstacles = Node(
        package="go2_navigation",
        executable="social_obstacle_publisher",
        name="social_obstacle_publisher",
        output="screen",
        parameters=[{
            # Fix 2: hard-code "map" because nav2_params_social.yaml sets
            # sensor_frame: map; using LaunchConfiguration(target_frame)
            # would create a mismatch if target_frame != "map".
            "frame_id": "map",
            # 5Hz × 8 ring points × <=2 persons in demo = ≤80 points
            # per cloud. Negligible bandwidth; default knobs are fine.
        }],
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        target_frame_arg, base_frame_arg,
        classes_arg, conf_arg, nav2_params_arg,
        nl_known_classes_arg, tf_grace_arg,

        LogInfo(msg=[
            "[command_first_demo] command-first online exploration. ",
            "Publish a string on /user_command to start the FSM. ",
            "Watch /task_coordinator/state for the FSM trace.",
        ]),
        LogInfo(msg=[
            "[command_first_demo] nav2_params=", nav2_params_file,
            " yoloe_classes=", classes,
            " tf_startup_grace_sec=", tf_startup_grace_sec, "s",
        ]),

        tf_and_scan,
        nav2_stack,
        yoloe_node, depth_projector, semantic_memory,
        frontier_node,
        target_selector, approach_planner, coordinator_node,
        nl_parser,
        social_obstacles,
    ])
