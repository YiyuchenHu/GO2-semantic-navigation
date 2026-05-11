"""Day 4 Nav2 launch — localization on the warehouse map.

Two backends are supported, switched via the `slam` launch arg:

  * slam=False (DEFAULT FOR REAL ROBOT): AMCL + map_server. Fast at
    runtime, doesn't change the map. Requires accurate, reliable
    /scan — AMCL silently stops broadcasting map→odom if its message
    filter drops a scan whose stamp predates the latest TF in cache,
    which has bitten us hard on Isaac Sim where the RTX LiDAR runs
    at ~4 Hz with occasional 14 s stalls under GPU contention.

  * slam=True (DEFAULT FOR SIM): slam_toolbox in online_async
    mapping mode replaces AMCL + map_server. slam_toolbox is far
    more tolerant of slow / jittery /scan because it accepts
    arbitrary scan-stamp ordering and rebuilds the local map every
    cycle. It publishes both /map and map→odom, so Nav2's costmaps
    and bt_navigator transparently consume it. We already use this
    backend in Day 3 and verified it handles our sim's /scan
    perfectly. The only "downside" is slam_toolbox keeps refining
    the map as Go2 drives — fine for navigation testing, would
    matter only if we cared about a fully static reference map.

Composition:
  1. Includes nav2_bringup's `bringup_launch.py` with our project
     params. bringup_launch starts (with slam=True):
       slam_toolbox (mapping mode, replaces amcl+map_server),
       controller_server, planner_server, behavior_server,
       bt_navigator, velocity_smoother, smoother_server,
       waypoint_follower, collision_monitor, docking_server,
       route_server
     all wrapped in a single `nav2_container` (use_composition=True),
     and runs them through `nav2_lifecycle_manager` so all of them go
     unconfigured → inactive → active automatically (autostart=True).
  2. Starts a `topic_tools/relay` node that bridges /cmd_vel_smoothed
     → /cmd_vel (only if `cmd_vel_relay:=true`; default is the
     collision_monitor pass-through bridge).

Why the cmd_vel bridge? In Jazzy, nav2_bringup's navigation_launch.py
applies this remap to controller_server, behavior_server,
velocity_smoother:
    cmd_vel → cmd_vel_nav
The chain becomes:
    controller_server   ──► /cmd_vel_nav
    behavior_server     ──► /cmd_vel_nav
    velocity_smoother  ──┬─► /cmd_vel_smoothed (output topic, NOT remapped)
                        └── (subscribes to cmd_vel_nav)
NOTHING publishes to /cmd_vel by default. Our sim's SubTwist node
subscribes to /cmd_vel — without a bridge, Go2 receives no commands.

We use Nav2's `collision_monitor` in pass-through mode (empty polygons)
as the bridge, configured in nav2_params.yaml:
    cmd_vel_in_topic:  cmd_vel_smoothed
    cmd_vel_out_topic: cmd_vel
This is the standard Nav2 way — collision_monitor sits between
Nav2's smoothed output and the wire. With empty polygons it never
modulates the command, so it's a no-op pipe in MVP. The
`cmd_vel_relay` launch arg is provided as a fallback (default off)
in case someone disables collision_monitor and needs a topic_tools
relay as a quick replacement.

Prerequisites:
  * apt: ros-jazzy-navigation2 ros-jazzy-nav2-bringup
         ros-jazzy-topic-tools
  * Phase 0 sim (`bash scripts/run_warehouse_ros2.sh`)
  * chair_perception (`ros2 launch go2_bringup_sim chair_perception.launch.py`)
    for /tf_static + /scan
  * Day 3 saved map (`maps/warehouse_v1.yaml`) — produced by
    scripts/save_map.sh after a clean SLAM run

Slam_toolbox MUST NOT be running concurrently. AMCL and slam_toolbox
both publish `map → odom`; running both makes the TF tree fight.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("go2_bringup_sim")
    nav2_share = get_package_share_directory("nav2_bringup")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")

    project_root = os.environ.get(
        "PROJECT_ROOT",
        # Fallback: 4 levels up from share/go2_bringup_sim →
        # install/<pkg>/share/<pkg>/. That'll usually be the
        # workspace root, where `maps/` lives.
        os.path.abspath(os.path.join(pkg_share, "..", "..", "..", "..")),
    )

    default_map = os.path.join(project_root, "maps", "warehouse_v1.yaml")
    default_params = os.path.join(pkg_share, "config", "nav2",
                                  "nav2_params.yaml")
    default_slam_params = os.path.join(pkg_share, "config", "slam",
                                       "slam_toolbox_mapping.yaml")

    slam_arg = DeclareLaunchArgument(
        "slam",
        # Default True for the sim because AMCL is unreliable here
        # (see module docstring). Override to False on real robot.
        default_value="True",
        description="If True, run slam_toolbox in online_async mapping "
                    "mode (publishes /map + map→odom). If False, run "
                    "AMCL + map_server with the static .yaml/.pgm.",
    )
    slam_params_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=default_slam_params,
        description="Path to slam_toolbox params (used only when slam:=True). "
                    "Default is the project-tuned config from Day 3.",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to the .yaml file describing the static "
                    "occupancy grid (used only when slam:=False). "
                    "Default is <PROJECT_ROOT>/maps/warehouse_v1.yaml.",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to the Nav2 params YAML. Default is the "
                    "project-tuned config under "
                    "go2_bringup_sim/config/nav2/nav2_params.yaml.",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Auto-activate every Nav2 lifecycle node at boot. "
                    "Set false if you want to drive transitions manually "
                    "via /lifecycle_manager_navigation/manage_nodes.",
    )
    use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Run all Nav2 nodes inside a single component "
                    "container (nav2_container). Faster, lower latency.",
    )
    use_relay_arg = DeclareLaunchArgument(
        "cmd_vel_relay",
        # Default OFF: the cmd_vel bridge is now done by Nav2's
        # collision_monitor in pass-through mode (empty polygons). It
        # subscribes to /cmd_vel_smoothed and republishes onto /cmd_vel.
        # See nav2_params.yaml's `collision_monitor` section. Two
        # things racing to publish /cmd_vel breaks sim's SubTwist, so
        # leave only one in the chain. Set this to true ONLY if you
        # disable collision_monitor entirely.
        default_value="false",
        description="If true, start a topic_tools/relay that "
                    "republishes /cmd_vel_smoothed onto /cmd_vel. "
                    "Default false because collision_monitor already "
                    "performs that bridge in pass-through mode.",
    )

    slam_flag = LaunchConfiguration("slam")
    slam_params_file = LaunchConfiguration("slam_params_file")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    cmd_vel_relay_flag = LaunchConfiguration("cmd_vel_relay")

    # ---- Nav2 stack + SLAM (slam:=True path) ------------------------------
    # IMPORTANT: when slam:=True we DO NOT use nav2_bringup's
    # bringup_launch.py. That file's slam_launch.py hardcodes
    # `online_sync_launch.py` (see /opt/ros/jazzy/share/nav2_bringup/launch/
    # slam_launch.py line 44), which spawns sync_slam_toolbox_node. The
    # sync variant runs scan-matching + Ceres pose-graph optimisation +
    # /tf publishing on a single thread, so any 5-10 s loop-closure or
    # heavy optimisation cycle stops broadcasting `map → odom` for the
    # whole duration. With Isaac Sim driving Go2, that gap is regularly
    # >5 s, which makes Nav2's controller_server abort every goal with
    # "Lookup would require extrapolation into the future" and the BT
    # then aborts to the client (perimeter_patrol / task_coordinator).
    #
    # The fix is mode, not tuning: async_slam_toolbox_node decouples the
    # TF publisher from the optimiser, so `map → odom` keeps streaming
    # at `transform_publish_period` (we set 0.1 s in
    # slam_toolbox_mapping.yaml) regardless of how long the current
    # scan_matcher / Ceres pass takes. We bring it up via
    # `slam_toolbox/launch/online_async_launch.py` directly, and bring
    # up the Nav2 servers via `nav2_bringup/launch/navigation_launch.py`
    # (the same one bringup_launch.py would have used internally — minus
    # the sync slam_toolbox include).
    #
    # When slam:=False (real-robot localisation path), we keep the
    # original bringup_launch.py because it correctly skips slam_launch
    # in that case and goes through localization_launch.py (amcl +
    # map_server) — no sync slam involvement, no need for special
    # handling.
    slam_toolbox_async = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, "launch",
                         "online_async_launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": slam_params_file,
            "autostart": autostart,
        }.items(),
        condition=IfCondition(slam_flag),
    )

    # IMPORTANT: navigation_launch.py with use_composition=True only
    # calls LoadComposableNodes against an EXISTING container — it does
    # NOT create the container itself. bringup_launch.py normally
    # creates `nav2_container` (rclcpp_components/component_container_
    # isolated) right before including navigation_launch.py. Since we
    # bypass bringup_launch.py to avoid its sync_slam dependency, we
    # have to spawn the container ourselves; otherwise LoadComposable-
    # Nodes silently no-ops because target_container='/nav2_container'
    # never exists, and /controller_server, /bt_navigator,
    # /navigate_to_pose etc never come up. The first iteration of this
    # fix bit us exactly that way (perimeter_patrol failed with
    # "/navigate_to_pose not available").
    #
    # parameters / remappings here mirror what bringup_launch.py would
    # have applied (see /opt/ros/jazzy/share/nav2_bringup/launch/
    # bringup_launch.py around line 146): the nav2 params file is
    # loaded onto the container so all composable nodes inherit it,
    # and /tf, /tf_static are remapped to relative names which is the
    # nav2 convention so they work consistently across namespaces.
    nav2_container = Node(
        package="rclcpp_components",
        executable="component_container_isolated",
        name="nav2_container",
        parameters=[params_file, {"autostart": autostart}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        output="screen",
        condition=IfCondition(slam_flag),
    )
    nav2_navigation_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "container_name": "nav2_container",
            "namespace": "",
        }.items(),
        condition=IfCondition(slam_flag),
    )

    # ---- AMCL + Nav2 full stack (slam:=False path) ------------------------
    # Real-robot path. bringup_launch.py with slam:=False uses
    # localization_launch.py (amcl + map_server) which doesn't have the
    # sync_slam_toolbox issue. Keep this path identical to the previous
    # behaviour.
    nav2_bringup_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam": "False",
            "map": map_yaml,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_localization": "True",
            "use_namespace": "false",
            "namespace": "",
        }.items(),
        condition=UnlessCondition(slam_flag),
    )

    # ---- /cmd_vel_smoothed → /cmd_vel relay --------------------------------
    # `topic_tools/relay` accepts positional args `<input> <output>`.
    # IfCondition gates this on the `cmd_vel_relay` launch flag.
    cmd_vel_relay_node = Node(
        package="topic_tools",
        executable="relay",
        name="cmd_vel_smoothed_to_cmd_vel",
        arguments=["/cmd_vel_smoothed", "/cmd_vel"],
        output="screen",
        condition=IfCondition(cmd_vel_relay_flag),
    )

    return LaunchDescription([
        # Defensive: if anyone forgets to set use_sim_time at the
        # bringup level, this catches the entire launch tree.
        SetParameter(name="use_sim_time", value=True),
        slam_arg,
        slam_params_arg,
        map_arg,
        params_arg,
        autostart_arg,
        use_composition_arg,
        use_relay_arg,
        LogInfo(msg=["[nav2.launch] slam=", slam_flag,
                     " (True → slam_toolbox replaces amcl + map_server)"]),
        LogInfo(msg=["[nav2.launch] slam_params=", slam_params_file]),
        LogInfo(msg=["[nav2.launch] map=", map_yaml,
                     " (used only when slam:=False)"]),
        LogInfo(msg=["[nav2.launch] params=", params_file]),
        LogInfo(msg=["[nav2.launch] cmd_vel relay=", cmd_vel_relay_flag,
                     " (relays /cmd_vel_smoothed → /cmd_vel)"]),
        LogInfo(msg=["[nav2.launch] SLAM mode: ", slam_flag,
                     " (True → async_slam_toolbox_node started directly, "
                     "bypassing nav2_bringup/slam_launch.py which would "
                     "force the sync variant)"]),
        slam_toolbox_async,
        nav2_container,
        nav2_navigation_only,
        nav2_bringup_amcl,
        cmd_vel_relay_node,
    ])
