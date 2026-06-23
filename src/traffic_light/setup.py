from setuptools import setup

package_name = "traffic_light"

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
    description="Traffic light state classifier for the Robo404 phase 1 ROS2 pipeline.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "traffic_light_node = traffic_light.traffic_light_node:main",
        ],
    },
)
