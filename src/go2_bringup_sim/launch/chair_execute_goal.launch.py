"""Phase 3B launch: goal execution + arrival verification.

Composition:
  1. Includes `chair_goto_goal.launch.py` (Phase 3A — produces
     /semantic_goal/goal_pose and /semantic_query/selected_target).
  2. Starts the Phase 3B layer:
       * go2_navigation::nav_executor_node
            backend = simple_p_controller
            -> /cmd_vel
            -> /navigation/status
       * go2_navigation::arrival_verifier_node
            -> /arrival/status
            -> /user_guidance/message

Explicitly does NOT start:
  * search_manager_node     (Phase 3B+ — search behaviour)
  * task_coordinator_node   (Phase 3B+ — end-to-end loop)
  * safety_monitor_node     (later — safety integration)

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
        description="TF frame that goal_pose / arrival checks are expressed in.",
    )
    target_class_arg = DeclareLaunchArgument(
        "target_class",
        default_value="chair",
        description="Phase 3A default target class.",
    )
    backend_arg = DeclareLaunchArgument(
        "nav_backend",
        default_value="simple_p_controller",
        description="nav_executor_node backend. Phase 3B MVP = "
                    "simple_p_controller. Other values: 'nav2' (requires "
                    "Nav2), 'go2_velocity' (stub only).",
    )
    stop_radius_arg = DeclareLaunchArgument(
        "stop_radius_m",
        default_value="0.25",
        description="Controller stop radius. Should be TIGHTER than the "
                    "arrival verifier's per-class reach distance so that "
                    "REACHED happens before ARRIVED_CONFIRMED.",
    )
    heading_tol_arg = DeclareLaunchArgument(
        "arrival_heading_tol_deg",
        default_value="40.0",
        description="Arrival verifier heading tolerance (degrees).",
    )
    publish_zero_idle_arg = DeclareLaunchArgument(
        "controller_publish_zero_when_idle",
        default_value="true",
        description="If true (default), nav_executor's P controller "
                    "publishes zero Twist on /cmd_vel every tick while "
                    "IDLE. Phase 4 sets this to false so a search "
                    "layer can own /cmd_vel without contention. Leave "
                    "true for Phase 3B-only launches.",
    )

    global_frame = LaunchConfiguration("global_frame")
    target_class = LaunchConfiguration("target_class")
    nav_backend = LaunchConfiguration("nav_backend")
    stop_radius = LaunchConfiguration("stop_radius_m")
    heading_tol = LaunchConfiguration("arrival_heading_tol_deg")
    publish_zero_idle = LaunchConfiguration("controller_publish_zero_when_idle")

    phase3a_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory("go2_bringup_sim"),
                "launch",
                "chair_goto_goal.launch.py",
            ),
        ),
        launch_arguments={
            "global_frame": global_frame,
            "target_class": target_class,
        }.items(),
    )

    nav_executor = Node(
        package="go2_navigation",
        executable="nav_executor_node",
        name="nav_executor_node",
        output="screen",
        parameters=[{
            "backend": nav_backend,
            "log_period_sec": 2.0,
            "controller.rotate_threshold_rad": 0.35,
            "controller.stop_radius_m": stop_radius,
            "controller.goal_update_threshold_m": 0.15,
            "controller.max_linear": 0.40,
            "controller.max_angular": 0.80,
            "controller.k_linear": 0.80,
            "controller.k_angular": 1.20,
            "controller.loop_hz": 10.0,
            "controller.publish_zero_when_idle": publish_zero_idle,
        }],
    )

    arrival_verifier = Node(
        package="go2_navigation",
        executable="arrival_verifier_node",
        name="arrival_verifier_node",
        output="screen",
        parameters=[{
            "global_frame": global_frame,
            "base_frame": "base_link",
            "heading_tol_deg": heading_tol,
            "recent_visible_sec": 2.0,
            "log_period_sec": 2.0,
        }],
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        global_frame_arg,
        target_class_arg,
        backend_arg,
        stop_radius_arg,
        heading_tol_arg,
        publish_zero_idle_arg,
        phase3a_launch,
        nav_executor,
        arrival_verifier,
    ])
