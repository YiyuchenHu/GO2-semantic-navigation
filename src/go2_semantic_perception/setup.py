from setuptools import find_packages, setup

package_name = "go2_semantic_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="go2_team",
    maintainer_email="go2@example.com",
    description=(
        "Day 6+ depth reprojection + semantic memory + Day 7 "
        "target selection / approach-goal planning: "
        "vision_msgs/Detection2DArray -> Detection3DArray -> "
        "go2_msgs/SemanticEntityArray -> SelectedTarget -> "
        "Nav2 NavigateToPose. Replaces the legacy chair-only "
        "Phase 1-3A stack."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "depth_projector_node = "
            "go2_semantic_perception.depth_projector_node:main",
            "semantic_memory_aggregator_node = "
            "go2_semantic_perception.semantic_memory_aggregator_node:main",
            "target_selector_node = "
            "go2_semantic_perception.target_selector_node:main",
            "approach_goal_planner_node = "
            "go2_semantic_perception.approach_goal_planner_node:main",
        ],
    },
)
