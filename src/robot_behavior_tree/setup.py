from setuptools import setup
import os
from glob import glob

package_name = 'robot_behavior_tree'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sirius',
    maintainer_email='sirius@example.com',
    description='ROS2 Robot Behavior Tree System',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_tree_monitor = robot_behavior_tree.behavior_tree_monitor:main',
            'mission_demo = robot_behavior_tree.mission_demo:main',
        ],
    },
)
