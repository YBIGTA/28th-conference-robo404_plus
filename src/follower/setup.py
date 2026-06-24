from setuptools import setup
from glob import glob
import os

package_name = 'follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('worlds/*.world'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Nascarella Hishida',
    maintainer_email='gabrielnhn@ufpr.br',
    description='Have a differential drive robot follow a Robotrace track by using a camera.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower_node = follower.follower_node:main',
            'tester = follower.test_topic:main',
            'line_tracker_node = follower.line_tracker_node:main',
            'yolo_detector_node = follower.yolo_detector_node:main',
            'safety_arbiter_node = follower.safety_arbiter_node:main',
            'wasd_teleop_node = follower.wasd_teleop_node:main',
            'rl_follower_node = follower.rl_follower_node:main'
        ],
    },
)
