import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wheel_odom_driver'

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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sirius',
    maintainer_email='sirius@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_test_node = wheel_odom_driver.serial_test_node:main',
            'unified_serial_manager_node = wheel_odom_driver.unified_serial_manager_node:main',
        ],
    },
)
