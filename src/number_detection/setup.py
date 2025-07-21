from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'number_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/number_detection.yaml']),
        ('share/' + package_name + '/launch', ['launch/number_detection.launch.py']),
        ('share/' + package_name + '/modules', []),  # 为模型文件预留目录
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sirius',
    maintainer_email='1710617204@qq.com',
    description='Number detection package for detecting numbers 30-60 using YOLO',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'number_detector_node = number_detection.number_detector_node:main',
        ],
    },
)
