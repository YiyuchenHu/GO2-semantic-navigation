from setuptools import find_packages, setup

package_name = "go2_task_coordinator"

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
    description="Task coordinator FSM for Go2 semantic navigation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_coordinator_node = go2_task_coordinator.task_coordinator_node:main",
        ],
    },
)
