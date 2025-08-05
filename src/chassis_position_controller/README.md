# 机器人位置控制器

## 🚀 系统概述

这是一个ROS2机器人位置控制器，提供精确的位置和速度控制功能。控制器修复了原有的cmd_vel输出问题，确保仿真和实物机器人都能正常执行运动任务。

## 📁 文件结构

```
chassis_position_controller/
├── correct_speed_controller.py     # 核心位置控制器
├── simple_robot_controller.py      # Python控制库
├── position_controller.launch.py   # 启动文件
└── start_position_controller.sh     # 快速启动脚本
```

## 🎯 快速开始

### 方法1：使用启动脚本（推荐）

```bash
# 进入工作空间
cd /home/sirius/ssd/ros2workspace

# 直接启动控制器
./src/chassis_position_controller/start_position_controller.sh

# 带仿真启动
./src/chassis_position_controller/start_position_controller.sh --sim

# 带硬件启动
./src/chassis_position_controller/start_position_controller.sh --hw

# 带可视化启动
./src/chassis_position_controller/start_position_controller.sh --sim --rviz
```

### 方法2：使用ROS2 launch文件

```bash
# 确保环境已正确source
source install/setup.bash

# 启动位置控制器
ros2 launch chassis_position_controller position_controller.launch.py
```

### 方法3：直接运行节点

```bash
ros2 run chassis_position_controller position_controller
```

## 🎮 控制机器人

### 基础位置控制

```bash
# 移动到位置 (1.0, 0.5)，朝向90度
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'odom'
pose:  
  position: {x: 1.0, y: 0.5, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
"
```

### 带速度的位置控制（推荐）

```bash
# 移动到位置 (2.0, 1.0)，朝向45度，线速度0.5m/s
ros2 topic pub --once /target_pose_with_speed geometry_msgs/msg/TwistStamped "
header:
  frame_id: 'odom'
twist:
  linear: {x: 2.0, y: 1.0, z: 0.5}    # x,y=位置, z=线速度
  angular: {z: 0.785, x: 0.0, y: 0.0}  # z=目标yaw(弧度)
"
```

### 速度限制控制

```bash
# 设置全局速度限制为0.3m/s
ros2 topic pub --once /target_speed_override std_msgs/msg/Float32 "data: 0.3"

# 清除速度限制
ros2 topic pub --once /target_speed_override std_msgs/msg/Float32 "data: -1.0"
```

## 🐍 Python脚本控制

### 使用简化控制库

```python
#!/usr/bin/env python3
from simple_robot_controller import SimpleRobotController

# 创建控制器
robot = SimpleRobotController()

try:
    # 启动
    robot.start()
    
    # 移动到指定位置
    robot.move_to(1.0, 0.5, yaw_deg=90, speed=0.3)
    
    # 向前移动
    robot.move_forward(0.5, speed=0.2)
    
    # 转向
    robot.turn_to(180)
    
    # 执行路径
    path = [(2.0, 0.0, 0), (2.0, 1.0, 90), (0.0, 1.0, 180)]
    robot.execute_path(path, speed=0.4)
    
finally:
    robot.stop()
```

### 在你的程序中集成

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math

class MyRobotProgram(Node):
    def __init__(self):
        super().__init__('my_robot_program')
        
        # 创建发布器
        self.target_pub = self.create_publisher(
            TwistStamped, '/target_pose_with_speed', 10)
    
    def send_robot_to(self, x, y, yaw_deg, speed=0.3):
        """发送机器人到指定位置"""
        msg = TwistStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.twist.linear.x = x      # 目标X
        msg.twist.linear.y = y      # 目标Y  
        msg.twist.linear.z = speed  # 移动速度
        
        msg.twist.angular.z = math.radians(yaw_deg)  # 目标朝向
        
        self.target_pub.publish(msg)
        self.get_logger().info(f'发送目标: ({x}, {y}, {yaw_deg}°) @ {speed}m/s')

# 使用示例
def main():
    rclpy.init()
    robot_program = MyRobotProgram()
    
    # 发送移动指令
    robot_program.send_robot_to(1.0, 0.0, 0, speed=0.4)
    
    rclpy.spin(robot_program)

if __name__ == '__main__':
    main()
```

## 📊 系统监控

```bash
# 查看里程计信息
ros2 topic echo /chassis/odom

# 查看速度指令输出
ros2 topic echo /cmd_vel

# 查看目标到达状态
ros2 topic echo /position_controller/target_reached

# 查看当前速度
ros2 topic echo /position_controller/current_speed
```

## 🛠️ 话题接口

### 输入话题
- `/target_pose` - 基础位置目标 (geometry_msgs/PoseStamped)
- `/target_pose_with_speed` - 带速度的位置目标 (geometry_msgs/TwistStamped)
- `/target_speed_override` - 速度覆盖 (std_msgs/Float32)
- `/chassis/odom` - 机器人里程计 (nav_msgs/Odometry)

### 输出话题
- `/cmd_vel` - 速度指令 (geometry_msgs/Twist)
- `/position_controller/target_reached` - 目标到达状态 (std_msgs/Bool)
- `/position_controller/current_speed` - 当前速度 (std_msgs/Float32)

## ⚙️ 参数配置

```python
# 可配置参数
parameters=[
    {'position_tolerance': 0.1},      # 位置容差(米)
    {'angle_tolerance': 0.1},         # 角度容差(弧度)  
    {'max_linear_speed': 1.0},        # 最大线速度
    {'max_angular_speed': 2.0},       # 最大角速度
    {'default_linear_speed': 0.3},    # 默认线速度
    {'default_angular_speed': 1.0},   # 默认角速度
]
```

## 🐛 故障排除

```bash
# 检查控制器是否运行
ros2 node list | grep position_controller

# 检查cmd_vel是否有输出
ros2 topic echo /cmd_vel

# 检查目标是否发送成功  
ros2 topic echo /target_pose_with_speed

# 清除速度限制
ros2 topic pub --once /target_speed_override std_msgs/msg/Float32 "data: -1.0"
```

## 🎉 特性

- ✅ **可靠的速度控制**: 确保cmd_vel正确输出
- 🎮 **多种控制方式**: 话题控制和Python编程接口
- 🚀 **简单启动**: 一键启动脚本
- 📊 **实时反馈**: 状态监控和位置反馈
- 🐍 **易用接口**: 简化的Python控制库
- ⚙️ **灵活配置**: 可调节的控制参数

现在你可以：
1. 使用 `./start_position_controller.sh` 快速启动
2. 通过话题直接控制机器人移动
3. 使用Python库编写自动化脚本
4. 在你的程序中集成机器人控制功能

系统已完全可用，机器人能够准确执行移动指令！🤖✨
