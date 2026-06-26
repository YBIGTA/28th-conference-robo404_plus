from setuptools import setup

package_name = "yolo_debug"

setup(
    name=package_name,
    version="4.6.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Miguel Ángel González Santamarta",
    maintainer_email="mgons@unileon.es",
    description="YOLO debug visualization and headless stream node",
    license="GPL-3.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "debug_node = yolo_debug.debug_node:main",
        ],
    },
)
