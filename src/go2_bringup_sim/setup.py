from setuptools import find_packages, setup

package_name = "go2_bringup_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [
            "launch/sim_semantic_nav.launch.py",
            "launch/chair_perception.launch.py",
            "launch/chair_semantic_memory.launch.py",
            "launch/chair_goto_goal.launch.py",
            "launch/chair_execute_goal.launch.py",
            "launch/chair_with_search.launch.py",
            "launch/mapping.launch.py",
            "launch/nav2.launch.py",
            "launch/yoloe.launch.py",
            "launch/day6.launch.py",
            "launch/day7.launch.py",
            "launch/day8.launch.py",
            "launch/day8_two_phase.launch.py",
            "launch/tf_and_scan.launch.py",
        ]),
        (f"share/{package_name}/config", ["config/sim_interface_contract.yaml"]),
        (f"share/{package_name}/config/slam", [
            "config/slam/slam_toolbox_mapping.yaml",
            "config/slam/slam_toolbox_motion_smooth.yaml",
        ]),
        (f"share/{package_name}/config/nav2", [
            "config/nav2/nav2_params.yaml",
        ]),
        (f"share/{package_name}/rviz", [
            "rviz/go2_semantic_nav.rviz",
            "rviz/go2_motion_debug.rviz",
        ]),
        (f"share/{package_name}/urdf", ["urdf/go2_minimal.urdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="go2_team",
    maintainer_email="go2@example.com",
    description="Isaac Sim bringup for Go2 semantic navigation MVP.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
