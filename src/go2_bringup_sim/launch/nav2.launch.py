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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("go2_bringup_sim")
    nav2_share = get_package_share_directory("nav2_bringup")

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

    # ---- nav2_bringup full stack -------------------------------------------
    # When slam:=True, nav2_bringup includes its slam_launch.py which
    # starts slam_toolbox in mapping mode + publishes /map and map→odom,
    # AND skips amcl/map_server. When slam:=False, nav2_bringup's
    # localization_launch.py is used (amcl + map_server with the static
    # .yaml/.pgm).
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam": slam_flag,
            "slam_params_file": slam_params_file,
            "map": map_yaml,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_localization": "True",
            "use_namespace": "false",
            "namespace": "",
        }.items(),
    )

    # ---- /cmd_vel_smoothed → /cmd_vel relay --------------------------------
    # `topic_tools/relay` accepts positional args `<input> <output>`.
    # IfCondition gates this on the `cmd_vel_relay` launch flag.
    from launch.conditions import IfCondition
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
        nav2_bringup_launch,
        cmd_vel_relay_node,
    ])
