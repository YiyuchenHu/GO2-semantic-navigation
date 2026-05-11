from setuptools import find_packages, setup

package_name = "go2_perception"

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
    description="Go2 perception package with backend abstraction.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_node = go2_perception.perception_node:main",
            # Day 5: open-vocabulary YOLOE detector. Publishes
            # standard vision_msgs/Detection2DArray on /detections,
            # independent of the chair-only perception_node above.
            "yoloe_detector_node = "
            "go2_perception.yoloe_detector_node:main",
        ],
    },
)
