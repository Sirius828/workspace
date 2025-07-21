# 轮式里程计驱动包

这个包用于从STM32通过串口接收里程计数据并发布为ROS2的轮式里程计话题。

## 功能描述

- 通过UART1 (/d### 5. 启动cmd_vel转串口节点
```bash
ros2 launch wheel_odom_driver cmd_vel_to_serial.launch.xml
```

### 6. 测试cmd_vel功能
```bash
ros2 launch wheel_odom_driver cmd_vel_serial_test.launch.xml
```

### 7. 双向通信（同时接收里程计和发送cmd_vel）
```bash
ros2 launch wheel_odom_driver wheel_odom_bidirectional.launch.xml
```

### 8. 手动发送cmd_vel命令
```bash
python3 /home/sirius/ssd/ros2workspace/src/wheel_odom_driver/scripts/manual_cmd_vel.py
```

### 9. 查看话题数据
```bash
# 查看里程计数据
ros2 topic echo /wheel/odom

# 发送速度命令
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}"
```tyTHS0) 从STM32接收里程计数据
- 解析格式为 "x位移,y位移,偏航角,x速度,y速度,角速度" 的数据
- 发布 `/wheel/odom` 话题 (nav_msgs/Odometry)
- 可选择发布 odom->base_link 的TF变换
- 支持串口断线重连

## 节点说明

### wheel_odom_node
主要的里程计处理节点，负责：
- 读取串口数据
- 解析里程计信息
- 发布 `/wheel/odom` 话题
- 发布TF变换 (可选)

### cmd_vel_to_serial_node
速度命令转串口节点，负责：
- 订阅 `/cmd_vel` 话题
- 将速度命令转换为串口数据格式
- 发送速度命令到STM32
- 支持超时保护（超时自动发送零速度）

### serial_test_node
串口测试节点，用于：
- 测试串口连接
- 调试数据格式
- 查看原始接收数据

### cmd_vel_serial_test_node
cmd_vel串口测试节点，用于：
- 测试cmd_vel到串口的数据传输
- 自动发送测试速度命令
- 监听串口响应

## 参数配置

### 串口配置
- `serial_port`: 串口设备路径 (默认: "/dev/ttyTHS0")
- `baud_rate`: 波特率 (默认: 115200)
- `timeout`: 串口超时时间 (默认: 1.0秒)

### 坐标系配置
- `frame_id`: 机器人基坐标系 (默认: "base_link")
- `odom_frame_id`: 里程计坐标系 (默认: "odom")

### cmd_vel相关配置
- `cmd_vel_topic`: 订阅的cmd_vel话题名称 (默认: "/cmd_vel")
- `publish_rate`: 发送频率 (默认: 10.0 Hz)
- `zero_timeout`: 超时时间，超过此时间没收到命令就发送零速度 (默认: 2.0秒)

## 使用方法

### 1. 构建包
```bash
cd /home/sirius/ssd/ros2workspace
colcon build --packages-select wheel_odom_driver
source install/setup.bash
```

### 2. 测试串口连接
```bash
ros2 launch wheel_odom_driver serial_test.launch.xml
```

### 3. 启动里程计节点
```bash
ros2 launch wheel_odom_driver wheel_odom.launch.xml
```

### 4. 查看发布的话题
```bash
ros2 topic echo /wheel/odom
```

## STM32端数据格式

### 接收里程计数据（STM32发送给ROS2）
STM32需要通过串口发送以下格式的数据：
```
时间戳,x位移,y位移,偏航角,x速度,y速度,角速度\n
```

示例数据：
```
12345,1.23,0.45,0.78,0.5,0.0,0.1
```

### 发送速度命令（ROS2发送给STM32）
ROS2会向STM32发送以下格式的速度命令：
```
vx,vy,omega\n
```

示例数据：
```
0.500,0.000,0.100
```

其中：
- vx: x方向线速度 (m/s)
- vy: y方向线速度 (m/s)  
- omega: 角速度 (rad/s)

## 硬件连接

- STM32 UART_TX → Jetson Orin Nano UART1_RX (Pin 10)
- STM32 UART_RX → Jetson Orin Nano UART1_TX (Pin 8) 
- GND → GND

## 注意事项

1. 确保Jetson Orin Nano的UART1已启用
2. 检查串口权限：`sudo chmod 666 /dev/ttyTHS0`
3. 确保波特率与STM32端保持一致
4. 数据格式必须严格按照 "值1,值2,值3,值4,值5,值6" 的格式
5. 每行数据以换行符结尾

## 故障排除

### 串口权限问题
```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyTHS0
```

### 查看串口状态
```bash
ls -l /dev/ttyTHS*
dmesg | grep tty
```

### 手动测试串口
```bash
sudo minicom -D /dev/ttyTHS0 -b 115200
```
