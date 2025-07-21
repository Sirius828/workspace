# 🔌 统一串口管理节点调试指南

## 📋 文件说明

### 配置文件
- `config/unified_serial_debug_config.yaml` - 专用调试配置文件

### Launch文件
- `launch/unified_serial_debug.launch.xml` - XML版本单独启动文件
- `launch/unified_serial_debug.launch.py` - Python版本单独启动文件（推荐）

### 测试脚本
- `scripts/test_serial_manager.py` - 交互式测试脚本

## 🚀 使用方法

### 1. 基础启动

#### XML版本
```bash
ros2 launch wheel_odom_driver unified_serial_debug.launch.xml
```

#### Python版本（推荐）
```bash
ros2 launch wheel_odom_driver unified_serial_debug.launch.py
```

### 2. 自定义参数启动

```bash
# 指定串口设备
ros2 launch wheel_odom_driver unified_serial_debug.launch.py serial_port:=/dev/ttyUSB0

# 指定波特率
ros2 launch wheel_odom_driver unified_serial_debug.launch.py baud_rate:=9600

# 启用调试日志
ros2 launch wheel_odom_driver unified_serial_debug.launch.py log_level:=debug

# 禁用TF发布（如果有其他里程计源）
ros2 launch wheel_odom_driver unified_serial_debug.launch.py enable_tf:=false

# 组合参数
ros2 launch wheel_odom_driver unified_serial_debug.launch.py \\
    serial_port:=/dev/ttyUSB0 \\
    baud_rate:=9600 \\
    log_level:=debug \\
    enable_tf:=false
```

### 3. 交互式测试

在另一个终端中启动测试脚本：
```bash
cd /home/sirius/ssd/ros2workspace
source install/setup.bash
python3 src/wheel_odom_driver/scripts/test_serial_manager.py
```

## 📊 监控和调试

### 查看话题列表
```bash
ros2 topic list
```

### 监控数据流

#### 里程计数据
```bash
ros2 topic echo /wheel/odom
```

#### 云台角度反馈
```bash
ros2 topic echo /gimbal/current_angle
```

#### 查看话题频率
```bash
ros2 topic hz /wheel/odom
ros2 topic hz /gimbal/current_angle
```

### 手动发送命令

#### 底盘控制
```bash
# 前进
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 左移
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 左转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# 停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

#### 云台控制
```bash
# 左转30度
ros2 topic pub /gimbal/angle_cmd geometry_msgs/msg/Vector3 '{x: 30.0, y: 0.0, z: 0.0}'

# 右转30度
ros2 topic pub /gimbal/angle_cmd geometry_msgs/msg/Vector3 '{x: -30.0, y: 0.0, z: 0.0}'

# 上仰20度
ros2 topic pub /gimbal/angle_cmd geometry_msgs/msg/Vector3 '{x: 0.0, y: 20.0, z: 0.0}'

# 下俯20度
ros2 topic pub /gimbal/angle_cmd geometry_msgs/msg/Vector3 '{x: 0.0, y: -20.0, z: 0.0}'

# 回中
ros2 topic pub /gimbal/angle_cmd geometry_msgs/msg/Vector3 '{x: 0.0, y: 0.0, z: 0.0}'
```

### 节点诊断

#### 查看节点信息
```bash
ros2 node info /unified_serial_manager
```

#### 查看参数
```bash
ros2 param list /unified_serial_manager
ros2 param get /unified_serial_manager serial_port
ros2 param get /unified_serial_manager baud_rate
```

#### 动态修改参数
```bash
ros2 param set /unified_serial_manager cmd_vel_timeout 5.0
```

## 🔧 故障排除

### 1. 串口连接问题
```bash
# 检查串口设备
ls -la /dev/tty*

# 检查串口权限
sudo chmod 666 /dev/ttyUSB0

# 检查串口占用
lsof /dev/ttyUSB0
```

### 2. 数据格式问题
确保STM32发送的数据格式为：
```
时间戳,x位移,y位移,偏航角,x速度,y速度,角速度,云台偏航角,云台俯仰角
```

例如：
```
12345,0.125,-0.050,0.785,0.500,-0.200,1.571,30.0,15.5
```

### 3. 无数据接收
- 检查串口设备路径
- 检查波特率设置
- 检查STM32是否正常发送数据
- 查看节点日志：`ros2 run wheel_odom_driver unified_serial_manager_node --ros-args --log-level debug`

### 4. 命令无响应
- 检查话题名称是否正确
- 确认串口写入权限
- 检查命令格式是否符合STM32期望

## 📈 性能监控

### CPU和内存使用
```bash
top -p $(pgrep -f unified_serial_manager)
```

### 网络流量（ROS话题）
```bash
ros2 topic bw /wheel/odom
ros2 topic bw /gimbal/current_angle
```

## 🎯 测试检查清单

- [ ] 串口连接正常
- [ ] 接收STM32数据并解析成功
- [ ] 里程计话题发布正常 (`/wheel/odom`)
- [ ] 云台角度话题发布正常 (`/gimbal/current_angle`)
- [ ] 底盘速度命令响应正常 (`/cmd_vel`)
- [ ] 云台角度命令响应正常 (`/gimbal/angle_cmd`)
- [ ] 命令超时机制工作正常
- [ ] TF变换发布正常（如启用）
- [ ] 无内存泄漏或异常错误

使用这些工具可以全面调试和测试统一串口管理节点的功能！🎉
