"""Phase 3A launch: target selection + approach goal generation.

Composition:
  1. Includes `chair_semantic_memory.launch.py` (Phase 2 — produces
     /semantic_map/entities).
  2. Starts the Phase 3A layer:
       * go2_navigation::target_selector_node
            -> /semantic_query/selected_target
            -> /semantic_query/selected_target_marker
       * go2_navigation::goal_planner_node
            -> /semantic_goal/goal_pose
            -> /semantic_goal/goal_candidates

Explicitly does NOT start:
  * nav_executor_node       (Phase 3B — actual driving)
  * arrival_verifier_node   (Phase 3B — arrival checks)
  * search_manager_node     (Phase 3B+ — search behaviour)
  * task_coordinator_node   (Phase 3B+ — end-to-end loop)
  * safety_monitor_node     (Phase 3B+ — safety integration)

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
        description="TF frame that entities / selections / goals are expressed "
                    "in. Phase 3A default 'odom' matches the Phase 0 / 2 TF "
                    "tree. Switch to 'map' once SLAM is online.",
    )
    target_class_arg = DeclareLaunchArgument(
        "target_class",
        default_value="chair",
        description="Phase 3A default target class for target_selector_node "
                    "when no /semantic_task/request is published.",
    )
    cost_threshold_arg = DeclareLaunchArgument(
        "cost_threshold",
        default_value="60",
        description="Occupancy value below which a goal candidate is still "
                    "considered safe. In the current sim there is no costmap, "
                    "so `safe_cost(None, ...)` returns True and all candidates "
                    "pass — this param only matters once Phase 3B+ brings a "
                    "real costmap publisher online.",
    )
    num_samples_arg = DeclareLaunchArgument(
        "num_angle_samples",
        default_value="16",
        description="Number of evenly-spaced approach candidates around the "
                    "chair. 16 is plenty for MVP.",
    )

    global_frame = LaunchConfiguration("global_frame")
    target_class = LaunchConfiguration("target_class")
    cost_threshold = LaunchConfiguration("cost_threshold")
    num_samples = LaunchConfiguration("num_angle_samples")

    # Phase 2 launch (semantic memory, which in turn includes Phase 1
    # perception). We forward `global_frame` and `target_class` so the
    # whole stack runs in one frame.
    phase2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory("go2_bringup_sim"),
                "launch",
                "chair_semantic_memory.launch.py",
            ),
        ),
        launch_arguments={
            "global_frame": global_frame,
            "target_class": target_class,
        }.items(),
    )

    target_selector = Node(
        package="go2_navigation",
        executable="target_selector_node",
        name="target_selector_node",
        output="screen",
        parameters=[{
            "global_frame": global_frame,
            "base_frame": "base_link",
            "default_target_class": target_class,
            "log_period_sec": 1.0,
            "select_period_sec": 0.5,
        }],
    )

    goal_planner = Node(
        package="go2_navigation",
        executable="goal_planner_node",
        name="goal_planner_node",
        output="screen",
        parameters=[{
            "global_frame": global_frame,
            "base_frame": "base_link",
            "num_angle_samples": num_samples,
            "cost_threshold": cost_threshold,
            "log_period_sec": 1.0,
        }],
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        global_frame_arg,
        target_class_arg,
        cost_threshold_arg,
        num_samples_arg,
        phase2_launch,
        target_selector,
        goal_planner,
    ])
