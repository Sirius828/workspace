from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gimbal_pixel_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sirius',
    maintainer_email='1710617204@qq.com',
    description='Gimbal pixel-based tracking controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pixel_controller_node = gimbal_pixel_controller.pixel_controller_node:main',
            'test_node = gimbal_pixel_controller.test_node:main',
            'performance_monitor = gimbal_pixel_controller.performance_monitor:main',
            'qos_diagnostic = gimbal_pixel_controller.qos_diagnostic:main',
        ],
    },
)
