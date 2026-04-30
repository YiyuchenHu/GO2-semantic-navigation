from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            Node(package="go2_command_interface", executable="command_parser_node", output="screen"),
            Node(package="go2_perception", executable="perception_node", output="screen"),
            Node(package="go2_object_localization", executable="object_localizer_3d_node", output="screen"),
            Node(package="go2_semantic_memory", executable="object_tracker_node", output="screen"),
            Node(package="go2_semantic_memory", executable="semantic_map_node", output="screen"),
            Node(package="go2_navigation", executable="target_selector_node", output="screen"),
            Node(package="go2_navigation", executable="goal_planner_node", output="screen"),
            Node(package="go2_navigation", executable="nav_executor_node", output="screen"),
            Node(package="go2_navigation", executable="search_manager_node", output="screen"),
            Node(package="go2_navigation", executable="arrival_verifier_node", output="screen"),
            Node(package="go2_task_coordinator", executable="task_coordinator_node", output="screen"),
            Node(package="go2_safety", executable="safety_monitor_node", output="screen"),
            Node(package="go2_debug_tools", executable="semantic_markers_node", output="screen"),
            Node(package="go2_debug_tools", executable="runtime_logger_node", output="screen"),
        ]
    )
