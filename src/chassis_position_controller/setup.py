from setuptools import setup
import os
from glob import glob

package_name = 'chassis_position_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sirius',
    maintainer_email='1710617204@qq.com',
    description='Position controller for chassis robot with speed control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_controller = chassis_position_controller.correct_speed_controller:main',
            'simple_robot_controller = chassis_position_controller.simple_robot_controller:main',
        ],
    },
)
