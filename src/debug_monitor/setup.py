from setuptools import setup

package_name = "debug_monitor"

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
    description="Topic-based debug monitor for the Robo404 phase 1 pipeline.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "debug_monitor_node = debug_monitor.debug_monitor_node:main",
        ],
    },
)
