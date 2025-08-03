import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sirius/ssd/ros2workspace/src/install/chassis_simulation'
