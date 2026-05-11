"""Phase 2 launch: chair perception + semantic memory.

Composition:
  1. Includes `chair_perception.launch.py` (Phase 1 — produces
     /perception/objects_3d).
  2. Starts the Phase 2 layer:
       * go2_semantic_memory::object_tracker_node
            -> /semantic/tracked_objects
       * go2_semantic_memory::semantic_map_node
            -> /semantic_map/entities
            -> /semantic_map/markers

Explicitly does NOT start:
  * target_selector_node
  * goal_planner_node
  * nav_executor_node
  * arrival_verifier_node
  * task_coordinator_node
  * safety_monitor_node
All of the above belong to later phases.

Prerequisite:
  Phase 0 sim must already be running
  (`bash scripts/run_warehouse_ros2.sh`).
"""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    global_frame_arg = DeclareLaunchArgument(
        "global_frame",
        default_value="odom",
        description="TF frame that tracks, entities and markers are expressed "
                    "in. Phase 2 default 'odom' matches the Phase 0 sim TF "
                    "tree. Switch to 'map' once SLAM is available.",
    )
    target_class_arg = DeclareLaunchArgument(
        "target_class",
        default_value="chair",
        description="Phase 2 default target class, forwarded to the chair "
                    "perception launch.",
    )
    assoc_dist_arg = DeclareLaunchArgument(
        "entity_association_distance_m",
        default_value="1.2",
        description="Max 3D distance (m) between a new track and an existing "
                    "semantic entity of the same class for re-association. "
                    "Making this too small will create duplicate entities "
                    "after every short perception dropout; too large will "
                    "merge distinct chairs into one.",
    )
    promote_n_arg = DeclareLaunchArgument(
        "promotion_min_observations",
        default_value="3",
        description="Minimum track.observations_count required before a "
                    "track can be promoted to a persistent semantic entity.",
    )
    promote_conf_arg = DeclareLaunchArgument(
        "promotion_min_confidence",
        default_value="0.45",
        description="Minimum track.confidence required before a track can "
                    "be promoted to a persistent semantic entity.",
    )

    global_frame = LaunchConfiguration("global_frame")
    target_class = LaunchConfiguration("target_class")
    assoc_dist = LaunchConfiguration("entity_association_distance_m")
    promote_n = LaunchConfiguration("promotion_min_observations")
    promote_conf = LaunchConfiguration("promotion_min_confidence")

    # Phase 1 perception launch (static TF + perception + localizer).
    phase1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory("go2_bringup_sim"),
                "launch",
                "chair_perception.launch.py",
            ),
        ),
        launch_arguments={
            "target_class": target_class,
            "global_frame": global_frame,
        }.items(),
    )

    tracker = Node(
        package="go2_semantic_memory",
        executable="object_tracker_node",
        name="object_tracker_node",
        output="screen",
        parameters=[{
            "global_frame": global_frame,
            # Keep tracker association tight — this is per-frame association
            # against the latest observations, not across dropouts. Entity
            # dropout tolerance lives in semantic_map_node below.
            "association_distance_m": 1.0,
            "ema_alpha": 0.4,
            "static_ttl_sec": 60.0,
            "log_period_sec": 1.0,
        }],
    )

    semantic_map = Node(
        package="go2_semantic_memory",
        executable="semantic_map_node",
        name="semantic_map_node",
        output="screen",
        parameters=[{
            "global_frame": global_frame,
            "promotion_min_observations": promote_n,
            "promotion_min_confidence": promote_conf,
            "ema_alpha": 0.35,
            "static_entity_ttl_sec": 180.0,
            "entity_association_distance_m": assoc_dist,
            "log_period_sec": 1.0,
        }],
    )

    return LaunchDescription([
        # Phase 0 sim drives /clock; every node below uses sim time.
        SetParameter(name="use_sim_time", value=True),
        global_frame_arg,
        target_class_arg,
        assoc_dist_arg,
        promote_n_arg,
        promote_conf_arg,
        phase1_launch,
        tracker,
        semantic_map,
    ])
