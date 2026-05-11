"""Day 8 — Frontier exploration on top of Day 7.

Builds on day7.launch.py via IncludeLaunchDescription so the five Day-7
nodes (yoloe, depth_projector, semantic_memory, target_selector,
approach_goal_planner) are launched verbatim. Adds two more nodes:

  * frontier_explorer_node (go2_navigation)
        Subscribes /map, exposes service /get_frontiers
        (go2_msgs/srv/GetFrontiers), publishes /frontier_markers.

  * task_coordinator_node (go2_task_coordinator)
        EXPLORE state owns its own NavigateToPose action client and
        calls /get_frontiers when the target_class can't be found in
        /semantic_map/objects. Preempts on entity-class match.

What this launch does NOT start (same as Day 7)
------------------------------------------------
* The simulator (run scripts/run_warehouse_ros2.sh).
* nav2.launch.py — Nav2 must already be up; the EXPLORE state sends
  NavigateToPose actions to /navigate_to_pose, identical to Day 7's
  approach_goal_planner.
* slam_toolbox / map_server — frontier_explorer_node needs a /map.
  Run mapping.launch.py (or your live-SLAM stack) before issuing a
  task.

Tunable args (Day 8 additions on top of Day 7's args)
-----------------------------------------------------

    ros2 launch go2_bringup_sim day8.launch.py \\
        target_class:=chair \\
        min_cluster_size:=10 \\
        info_gain_radius_m:=1.5 \\
        distance_weight:=5.0 \\
        max_frontiers:=5

`target_class` is forwarded to BOTH approach_goal_planner (via Day 7)
AND task_coordinator's `default_target_class`, so a single launch arg
sets the same target across the whole stack.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Pass-through args: forwarded straight to day7.launch.py. Anything
    # day7 declares is still settable on the day8 command line.
    # ------------------------------------------------------------------
    target_class_arg = DeclareLaunchArgument(
        "target_class", default_value="chair",
        description="Forwarded to Day 7's approach planner AND used as "
                    "task_coordinator.default_target_class.",
    )
    target_frame_arg = DeclareLaunchArgument(
        "target_frame", default_value="map",
        description="Frame for /detections_3d, semantic memory, and "
                    "frontier output. Must match the SLAM /map frame.",
    )
    base_frame_arg = DeclareLaunchArgument(
        "base_frame", default_value="base_link"
    )

    # ------------------------------------------------------------------
    # Day 8 specific args
    # ------------------------------------------------------------------
    map_topic_arg = DeclareLaunchArgument(
        "map_topic", default_value="/map",
        description="OccupancyGrid topic frontier_explorer reads.",
    )
    min_cluster_size_arg = DeclareLaunchArgument(
        "min_cluster_size", default_value="10",
        description="Drop frontier cell-clusters smaller than this.",
    )
    info_gain_radius_arg = DeclareLaunchArgument(
        "info_gain_radius_m", default_value="1.5",
        description="Radius (m) around each frontier centroid in which "
                    "unknown cells are counted as 'info gain'.",
    )
    distance_weight_arg = DeclareLaunchArgument(
        "distance_weight", default_value="5.0",
        description="Score = info_gain - distance_weight * "
                    "distance_to_robot.",
    )
    max_frontiers_arg = DeclareLaunchArgument(
        "max_frontiers", default_value="5",
        description="Top-N frontiers returned per /get_frontiers call.",
    )
    safety_radius_arg = DeclareLaunchArgument(
        "safety_radius_m", default_value="0.4",
        description="Frontier centroid must be at least this many "
                    "metres away from the nearest occupied cell on /map. "
                    "Set slightly under nav2 inflation_radius (0.5 m) so "
                    "that returned goals are reachable. 0 disables.",
    )
    snap_search_radius_arg = DeclareLaunchArgument(
        "snap_search_radius_m", default_value="1.0",
        description="If the raw centroid is unsafe, search this radius "
                    "for a free + safe replacement cell before dropping "
                    "the cluster.",
    )
    costmap_topic_arg = DeclareLaunchArgument(
        "costmap_topic", default_value="/global_costmap/costmap",
        description="nav2 inflated costmap consumed by the TB3-style "
                    "centroid-safety predicate. Leave default if you "
                    "launched bringup with the standard nav2 stack.",
    )
    costmap_safe_max_cost_arg = DeclareLaunchArgument(
        "costmap_safe_max_cost", default_value="75",
        description="Centroid is rejected when costmap cost >= this. "
                    "75 follows the TB3 frontier paper.",
    )
    coord_log_period_arg = DeclareLaunchArgument(
        "coord_log_period_sec", default_value="2.0"
    )
    coord_tick_period_arg = DeclareLaunchArgument(
        "coord_tick_period_sec", default_value="0.2"
    )
    parse_fallback_arg = DeclareLaunchArgument(
        "parse_command_fallback_sec", default_value="1.5",
        description="If PARSE_COMMAND state lasts longer than this and "
                    "no /semantic_task/request was published by an "
                    "external parser, synthesize a SemanticTask from "
                    "default_target_class so the FSM can advance. "
                    "Set to 0 (or negative) to disable the fallback.",
    )

    target_class = LaunchConfiguration("target_class")
    target_frame = LaunchConfiguration("target_frame")
    base_frame = LaunchConfiguration("base_frame")
    map_topic = LaunchConfiguration("map_topic")
    min_cluster_size = LaunchConfiguration("min_cluster_size")
    info_gain_radius_m = LaunchConfiguration("info_gain_radius_m")
    distance_weight = LaunchConfiguration("distance_weight")
    max_frontiers = LaunchConfiguration("max_frontiers")
    safety_radius_m = LaunchConfiguration("safety_radius_m")
    snap_search_radius_m = LaunchConfiguration("snap_search_radius_m")
    costmap_topic = LaunchConfiguration("costmap_topic")
    costmap_safe_max_cost = LaunchConfiguration("costmap_safe_max_cost")
    coord_log_period_sec = LaunchConfiguration("coord_log_period_sec")
    coord_tick_period_sec = LaunchConfiguration("coord_tick_period_sec")
    parse_command_fallback_sec = LaunchConfiguration(
        "parse_command_fallback_sec"
    )

    # ------------------------------------------------------------------
    # Reuse Day 7 wholesale — no edits to that file are required.
    # IncludeLaunchDescription forwards unknown args through.
    # ------------------------------------------------------------------
    day7_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("go2_bringup_sim"),
                "launch",
                "day7.launch.py",
            ])
        ]),
        launch_arguments={
            "target_class": target_class,
            "target_frame": target_frame,
            "base_frame": base_frame,
        }.items(),
    )

    # ------------------------------------------------------------------
    # Day 8 nodes
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
            "marker_topic": "/frontier_markers",
            "marker_ns": "frontiers",
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
            "default_target_class": target_class,
            "get_frontiers_service": "/get_frontiers",
            "nav_action_name": "/navigate_to_pose",
            "tick_period_sec": coord_tick_period_sec,
            "log_period_sec": coord_log_period_sec,
            "parse_command_fallback_sec": parse_command_fallback_sec,
        }],
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        target_class_arg,
        target_frame_arg,
        base_frame_arg,
        map_topic_arg,
        min_cluster_size_arg,
        info_gain_radius_arg,
        distance_weight_arg,
        max_frontiers_arg,
        safety_radius_arg,
        snap_search_radius_arg,
        costmap_topic_arg,
        costmap_safe_max_cost_arg,
        coord_log_period_arg,
        coord_tick_period_arg,
        parse_fallback_arg,
        LogInfo(msg=["[day8.launch] Day 8 frontier exploration enabled. "
                     "target_class=", target_class,
                     " map_topic=", map_topic,
                     " min_cluster=", min_cluster_size,
                     " info_radius=", info_gain_radius_m,
                     " dist_w=", distance_weight,
                     " max_frontiers=", max_frontiers]),
        day7_include,
        frontier_node,
        coordinator_node,
    ])
