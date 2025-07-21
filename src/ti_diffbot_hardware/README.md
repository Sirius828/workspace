# TiDiffBot Hardware Interface

这是一个基于ROS2 ros2_control框架的差速轮小车硬件接口包。

## 功能特性

- 基于串口通信的硬件接口
- 支持位置和速度状态反馈
- 支持速度控制命令
- 集成差速轮控制器
- 支持里程计发布

## 硬件要求

- 差速轮小车底盘
- 串口通信接口（如USB转串口）
- 编码器反馈

## 通信协议

### 从硬件读取数据格式：
```
left_tick,right_tick,left_rpm,right_rpm
```

### 向硬件发送数据格式：
```
left_rpm,right_rpm
```

## 配置参数

主要参数在 `config/hardware_interface.yaml` 中配置：

- `device_path`: 串口设备路径，如 `/dev/ttyUSB0`
- `baud_rate`: 波特率，默认 115200
- `enc_ticks_per_rev`: 编码器每转刻度数
- `gear_ratio`: 减速比
- `wheel_radius`: 轮子半径（米）
- `wheel_separation`: 轮子间距（米）

## 编译和安装

1. 进入工作空间：
```bash
cd /home/sirius/ssd/ros2workspace
```

2. 编译包：
```bash
colcon build --packages-select ti_diffbot_hardware
```

3. 加载环境：
```bash
source install/setup.bash
```

## 使用方法

1. 启动硬件接口和控制器：
```bash
ros2 launch ti_diffbot_hardware diffbot_hardware.launch.py
```

2. 发送速度命令：
```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.5, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.0}"
```

3. 查看关节状态：
```bash
ros2 topic echo /joint_states
```

4. 查看里程计：
```bash
ros2 topic echo /diff_drive_controller/odom
```

## 测试

运行测试脚本：
```bash
ros2 run ti_diffbot_hardware test_diffbot.py
```

## 文件结构

```
ti_diffbot_hardware/
├── CMakeLists.txt
├── package.xml
├── ti_diffbot_hardware.xml
├── include/
│   └── ti_diffbot_hardware/
│       └── ti_diffbot_hardware.hpp
├── src/
│   └── ti_diffbot_hardware.cpp
├── config/
│   ├── diffbot_controllers.yaml
│   └── hardware_interface.yaml
├── launch/
│   └── diffbot_hardware.launch.py
├── urdf/
│   └── ti_diffbot.urdf.xacro
└── scripts/
    └── test_diffbot.py
```

## 故障排除

1. **串口权限问题**：
```bash
sudo chmod 666 /dev/ttyUSB0
# 或者将用户添加到dialout组
sudo usermod -a -G dialout $USER
```

2. **找不到串口设备**：
```bash
ls /dev/tty*
```

3. **检查硬件接口状态**：
```bash
ros2 control list_hardware_interfaces
```

4. **检查控制器状态**：
```bash
ros2 control list_controllers
```

## 许可证

Apache-2.0

## 作者

Your Name (your.email@example.com)
