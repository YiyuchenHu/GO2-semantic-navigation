from setuptools import find_packages, setup

package_name = "go2_debug_tools"

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
    description="Debug tools for semantic navigation runtime inspection.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "semantic_markers_node = go2_debug_tools.semantic_markers_node:main",
            "runtime_logger_node = go2_debug_tools.runtime_logger_node:main",
            "integration_command_publisher = go2_debug_tools.integration_command_publisher:main",
            "integration_trace_watcher = go2_debug_tools.integration_trace_watcher:main",
            "synthetic_chair_observation_publisher = go2_debug_tools.synthetic_chair_observation_publisher:main",
        ],
    },
)
