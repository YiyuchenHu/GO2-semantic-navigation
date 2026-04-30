from setuptools import find_packages, setup

package_name = "go2_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="go2_team",
    maintainer_email="go2@example.com",
    description="Semantic navigation components for Go2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "target_selector_node = go2_navigation.target_selector_node:main",
            "goal_planner_node = go2_navigation.goal_planner_node:main",
            "nav_executor_node = go2_navigation.nav_executor_node:main",
            "search_manager_node = go2_navigation.search_manager_node:main",
            "arrival_verifier_node = go2_navigation.arrival_verifier_node:main",
        ],
    },
)
