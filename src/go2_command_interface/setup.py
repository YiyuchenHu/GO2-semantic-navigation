from setuptools import find_packages, setup

package_name = "go2_command_interface"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/semantic_targets.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="go2_team",
    maintainer_email="go2@example.com",
    description="Command parsing and task request generation for Go2 semantic navigation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "command_parser_node = go2_command_interface.command_parser_node:main",
        ],
    },
)
