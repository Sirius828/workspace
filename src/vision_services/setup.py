from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_services'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/vision_config.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/vision_services.launch.py',
            'launch/vision_gui.launch.py',
            'launch/vision_services_with_gui.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Vision services package for detecting squares and rectangles',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_services_node = vision_services.vision_services_node:main',
            'vision_gui_node = vision_services.vision_gui_node:main',
            'test_client = vision_services.test_client:main',
            'preview_demo = vision_services.preview_demo:main',
        ],
    },
)
