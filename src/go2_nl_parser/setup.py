from setuptools import find_packages, setup

package_name = "go2_nl_parser"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="go2_team",
    maintainer_email="go2@example.com",
    description=(
        "Regex + keyword natural-language parser that turns "
        "/user_command strings into /semantic_task/request "
        "SemanticTask messages for the Day 8 Phase B operator console."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nl_parser_node = go2_nl_parser.nl_parser_node:main",
        ],
    },
)
