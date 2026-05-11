"""Phase 4 launch: Phase 3B execution + search / reacquisition.

Composition:
  1. Includes `chair_execute_goal.launch.py` (Phase 3B — which in turn
     includes Phases 1, 2, 3A). We force the P controller's
     `publish_zero_when_idle` to False so that while nav_executor is
     IDLE, Phase 4's search_manager_node can own /cmd_vel without
     being overwritten by a 10 Hz zero-Twist spam.
  2. Starts go2_navigation::search_manager_node with the Phase 4
     parameters (target class, recency window, sweep timeout, sweep
     angular rate).

Explicitly does NOT start:
  * go2_task_coordinator::task_coordinator_node  (later phase)
  * go2_safety::safety_monitor_node              (later phase)
  * frontier / SLAM exploration                  (later phase)

Prerequisite:
  Phase 0 sim must already be running
  (`bash scripts/run_warehouse_ros2.sh` or equivalent).
"""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    target_class_arg = DeclareLaunchArgument(
        "target_class",
        default_value="chair",
        description="Phase 4 target class (chair-only MVP).",
    )
    recent_visible_arg = DeclareLaunchArgument(
        "search_recent_visible_sec",
        default_value="2.0",
        description="How recently the chair must have been seen (either "
                    "via /semantic_map/entities.currently_visible or a "
                    "raw /perception/objects_3d hit) to count as "
                    "'still there'. Below this window, Phase 4 will "
                    "sweep.",
    )
    search_timeout_arg = DeclareLaunchArgument(
        "search_timeout_sec",
        default_value="30.0",
        description="Maximum sweep duration before Phase 4 declares "
                    "LOST and stops rotating. A fresh detection at any "
                    "time resets the state machine to REACQUIRED.",
    )
    search_w_arg = DeclareLaunchArgument(
        "search_angular_rate",
        default_value="0.4",
        description="Angular rate used in SEARCHING (rad/s). Half of "
                    "Phase 3B's max_angular by default so the scene "
                    "stays crisp for YOLO.",
    )
    global_frame_arg = DeclareLaunchArgument(
        "global_frame",
        default_value="odom",
        description="TF frame for search markers / coherent with "
                    "Phase 3A and 3B.",
    )

    target_class = LaunchConfiguration("target_class")
    recent_visible = LaunchConfiguration("search_recent_visible_sec")
    search_timeout = LaunchConfiguration("search_timeout_sec")
    search_w = LaunchConfiguration("search_angular_rate")
    global_frame = LaunchConfiguration("global_frame")

    # Include Phase 3B launch, but force
    # controller_publish_zero_when_idle:=false so that while
    # nav_executor is IDLE it stays completely silent on /cmd_vel.
    # That hands /cmd_vel over to search_manager_node during search,
    # without any node duplication. This is a PARAMETER override —
    # Phase 3B's own default (true) is preserved when launched alone.
    phase3b_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory("go2_bringup_sim"),
                "launch",
                "chair_execute_goal.launch.py",
            ),
        ),
        launch_arguments={
            "global_frame": global_frame,
            "target_class": target_class,
            "controller_publish_zero_when_idle": "false",
        }.items(),
    )

    search_manager = Node(
        package="go2_navigation",
        executable="search_manager_node",
        name="search_manager_node",
        output="screen",
        parameters=[{
            "target_class": target_class,
            "recent_visible_sec": recent_visible,
            "search_timeout_sec": search_timeout,
            "search_angular_rate": search_w,
            "loop_hz": 10.0,
            "log_period_sec": 2.0,
            "global_frame": global_frame,
        }],
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        target_class_arg,
        recent_visible_arg,
        search_timeout_arg,
        search_w_arg,
        global_frame_arg,
        phase3b_launch,
        search_manager,
    ])
