# Chassis Hardware Package

全向轮底盘通信包，用于与底盘MCU进行串口通信。

## 功能描述

### 接收功能
- 从串口接收底盘里程计数据，格式：`x偏移，y偏移，累计偏航角，x速度，y速度，角速度\r`
- 将接收到的数据封装成ROS2里程计消息发布到 `/chassis/odom` 话题
- 发布TF变换 `odom` -> `base_link`

### 发送功能
- 订阅 `/cmd_vel` 话题，获取底盘运动指令
- 订阅 `/cmd_gimbal` 话题，获取云台角度指令
- 将指令合并后发送到串口，格式：`x速度，y速度，角速度，云台yaw角度，云台pitch角度\r`

## 参数配置

- `serial_port`: 串口设备路径 (默认: `/dev/ttyCH341USB0`)
- `baud_rate`: 波特率 (默认: `115200`)
- `base_frame_id`: 底盘坐标系名称 (默认: `base_link`)
- `odom_frame_id`: 里程计坐标系名称 (默认: `odom`)

## 编译和运行

### 编译
```bash
cd /home/sirius/ssd/ros2workspace
colcon build --packages-select chassis_hardware
source install/setup.bash
```

### 运行
```bash
# 使用默认参数运行
ros2 launch chassis_hardware chassis_hardware.launch.py

# 使用调试模式运行（显示详细的串口数据）
ros2 launch chassis_hardware chassis_hardware_debug.launch.py

# 或者使用自定义串口
ros2 launch chassis_hardware chassis_hardware.launch.py serial_port:=/dev/ttyUSB0

# 或者直接运行节点
ros2 run chassis_hardware chassis_hardware_node
```

## 调试和故障排除

### 启用调试日志
```bash
# 使用调试启动文件
ros2 launch chassis_hardware chassis_hardware_debug.launch.py

# 或者手动设置日志级别
ros2 run chassis_hardware chassis_hardware_node --ros-args --log-level chassis_hardware_node:=DEBUG
```

### 常见问题

1. **串口数据解析错误 (`stod` 错误)**
   - 检查MCU发送的数据格式是否为：`数字,数字,数字,数字,数字,数字\r`
   - 确保数据为纯数字，不包含其他字符
   - 使用调试模式查看接收到的原始数据

2. **串口权限问题**
   ```bash
   sudo chmod 666 /dev/ttyCH341USB0
   # 或者将用户添加到dialout组
   sudo usermod -a -G dialout $USER
   ```

3. **串口设备不存在**
   ```bash
   # 查看可用的串口设备
   ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH341USB*
   ```

## 话题接口

### 发布的话题
- `/chassis/odom` (nav_msgs/Odometry): 底盘里程计信息

### 订阅的话题
- `/cmd_vel` (geometry_msgs/Twist): 底盘运动控制指令
- `/cmd_gimbal` (geometry_msgs/Twist): 云台控制指令
  - `angular.x`: 云台yaw角度 (弧度)
  - `angular.y`: 云台pitch角度 (弧度)

## 数据格式

### 串口接收格式 (从MCU接收)
```
ODOM,x累计位移,y累计位移,累计偏航角,x速度,y速度,角速度\r\n
```
所有单位均为ROS2标准单位 (米、弧度、米/秒、弧度/秒)
- x累计位移: 从起始点到当前位置的x方向距离 (米)
- y累计位移: 从起始点到当前位置的y方向距离 (米)  
- 累计偏航角: 累计旋转角度 (弧度)
- x速度: 当前x方向速度 (米/秒)
- y速度: 当前y方向速度 (米/秒)
- 角速度: 当前角速度 (弧度/秒)

### 串口发送格式 (发送到MCU)
```
x速度,y速度,角速度,云台yaw角度(度),云台pitch角度(度)\r
```
- 底盘速度单位：米/秒、弧度/秒 (ROS2标准单位)
- 云台角度单位：角度制(度)，从ROS2标准的弧度制自动转换

## 注意事项

1. 确保串口设备权限正确：
   ```bash
   sudo chmod 666 /dev/ttyCH341USB0
   ```

2. 如果串口设备不存在，请检查硬件连接

3. 云台角度通过 `/cmd_gimbal` 话题的 `angular.x` (yaw) 和 `angular.y` (pitch) 字段传递 (弧度制)，程序会自动转换为角度制发送给MCU

## 依赖项

- rclcpp
- geometry_msgs
- nav_msgs
- tf2
- tf2_ros
- serial (ROS2串口库)
