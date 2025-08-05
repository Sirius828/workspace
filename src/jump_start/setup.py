from setuptools import setup

package_name = 'jump_start'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/jump_start.launch.py',
            'launch/integrated_system.launch.py'
        ]),
        ('share/' + package_name + '/config', ['config/fsm_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sirius',
    maintainer_email='sirius@todo.todo',
    description='Simple FSM-based robot control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_controller = jump_start.fsm_controller:main',
            'mission_executor = jump_start.mission_executor:main',
            'simple_demo = jump_start.simple_demo:main',
        ],
    },
)
