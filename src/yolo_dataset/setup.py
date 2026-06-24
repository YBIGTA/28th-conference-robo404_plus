import os
from glob import glob

from setuptools import setup

package_name = "yolo_dataset"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jiucai",
    maintainer_email="imshen19@gmail.com",
    description="Gazebo auto-labelling data pipeline for the Robo404 YOLO detector.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "recorder_node = yolo_dataset.recorder_node:main",
        ],
    },
)
