from setuptools import setup

package_name = "decision"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sleepy lee",
    maintainer_email="sleepy@example.com",
    description="Decision layer for the Robo404 phase 1 ROS2 pipeline.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "decision_node = decision.decision_node:main",
        ],
    },
)
