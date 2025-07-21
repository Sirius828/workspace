from setuptools import find_packages, setup

package_name = 'gimbal_trajectory_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/trajectory_planner.launch.py',
            'launch/simple_gimbal.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/trajectory_planner_config.yaml',
            'config/simple_gimbal_config.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sirius',
    maintainer_email='1710617204@qq.com',
    description='Gimbal trajectory planner for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_planner_node = gimbal_trajectory_planner.trajectory_planner_node:main',
        ],
    },
)
