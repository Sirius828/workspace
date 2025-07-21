from setuptools import setup, find_packages
import glob
setup(
    name='gimbal_pixel_mapper',
    version='0.1.0',
    description='Pixel to gimbal angle mapper for ROS 2.',
    author='Your Name',
    license='Apache-2.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'numpy',
        'opencv-python',   # 若只用 bilinear 可移除
    ],
    entry_points={
        'console_scripts': [
            'pixel_to_gimbal_node = gimbal_pixel_mapper.pixel_to_gimbal_node:main',
        ],
    },
    package_data={
        '': ['package.xml'],            # 安装顶层 package.xml
        'gimbal_pixel_mapper': ['resource/*'],  # 安装 resource/ 下的标记文件
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/gimbal_pixel_mapper']),
        ('share/gimbal_pixel_mapper', ['package.xml']),
        ('share/gimbal_pixel_mapper/launch', glob.glob('launch/*.py')),
        ('share/gimbal_pixel_mapper/config', glob.glob('config/*.yaml')),
    ],
    zip_safe=False,
)