import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'keyboard_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sirius',
    maintainer_email='1710617204@qq.com',
    description='Keyboard teleoperation package for robot control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop_node = keyboard_teleop.keyboard_teleop_node:main',
            'simple_keyboard_teleop_node = keyboard_teleop.simple_keyboard_teleop_node:main',
            'improved_keyboard_teleop_node = keyboard_teleop.improved_keyboard_teleop_node:main',
            'pygame_keyboard_teleop_node = keyboard_teleop.pygame_keyboard_teleop_node:main',
            'reliable_keyboard_teleop_node = keyboard_teleop.reliable_keyboard_teleop_node:main',
        ],
    },
)
