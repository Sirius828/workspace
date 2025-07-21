# Simple Navigation 包

基于里程计的简单机器人导航包，适用于差速轮机器人。

## 📋 功能特性

- ✅ 订阅 `/diff_drive_controller/odom` (里程计)
- ✅ 订阅 `/goal_pose` (目标位置)  
- ✅ 发布 `/diff_drive_controller/cmd_vel_unstamped` (速度命令)
- ✅ 三阶段导航控制（转向→前进→最终调整）
- ✅ 可调节PID参数
- ✅ 支持RViz2集成

## 🚀 快速开始

### 1. 编译包
```bash
cd ~/ssd/ros2workspace
colcon build --packages-select simple_navigation
source install/setup.bash
```

### 2. 启动完整系统（推荐）
```bash
# 启动底盘 + 导航器 + RViz
ros2 launch simple_navigation full_navigation_demo.launch.py simulation_mode:=true
```

### 3. 分别启动组件
```bash
# 终端1: 启动底盘硬件接口
ros2 launch ti_diffbot_hardware diffbot_hardware.launch.py simulation_mode:=true

# 终端2: 启动简单导航器
ros2 launch simple_navigation simple_navigation.launch.py
```

## 🎮 控制机器人

### 方法1: 使用RViz2 GUI
1. 在RViz中点击工具栏的 "2D Goal Pose" 
2. 在地图上点击目标位置
3. 拖拽设置机器人朝向
4. 机器人会自动导航到目标位置

### 方法2: 使用Python脚本
```bash
# 发送单个目标位置
ros2 run simple_navigation send_goal.py --x 2.0 --y 1.0 --yaw 90 --degrees

# 按正方形路径导航
ros2 run simple_navigation send_waypoints.py
```

### 方法3: 使用命令行
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'odom'
pose:
  position: {x: 2.0, y: 1.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}" --once
```

## ⚙️ 参数配置

### 控制参数
- `linear_kp`: 线速度比例增益 (默认: 1.0)
- `angular_kp`: 角速度比例增益 (默认: 2.0)
- `max_linear_vel`: 最大线速度 m/s (默认: 0.5)
- `max_angular_vel`: 最大角速度 rad/s (默认: 1.0)
- `position_tolerance`: 位置到达容差 m (默认: 0.1)
- `angle_tolerance`: 角度到达容差 rad (默认: 0.1)

### 自定义参数启动
```bash
ros2 launch simple_navigation simple_navigation.launch.py \
  linear_kp:=1.5 \
  angular_kp:=3.0 \
  max_linear_vel:=0.3 \
  max_angular_vel:=0.6
```

## 📊 话题接口

### 订阅话题
- `/diff_drive_controller/odom` (nav_msgs/msg/Odometry) - 机器人里程计
- `/goal_pose` (geometry_msgs/msg/PoseStamped) - 目标位置姿态

### 发布话题  
- `/diff_drive_controller/cmd_vel_unstamped` (geometry_msgs/msg/Twist) - 速度命令

## 🎯 导航算法

### 三阶段控制策略

1. **阶段1 - 初始转向**: 距离较远且朝向偏差大时，原地转向对准目标
2. **阶段2 - 导航前进**: 朝向正确后前进并微调方向
3. **阶段3 - 最终调整**: 到达目标位置后调整最终朝向

### 控制公式
```cpp
// 线速度控制
linear_velocity = linear_kp * distance * speed_factor
linear_velocity = clamp(linear_velocity, 0.05, max_linear_vel)

// 角速度控制  
angular_velocity = angular_kp * yaw_error
angular_velocity = clamp(angular_velocity, -max_angular_vel, max_angular_vel)
```

## 🔧 示例脚本

### send_goal.py 用法
```bash
# 基本用法
ros2 run simple_navigation send_goal.py --x 1.0 --y 0.5 --yaw 45 --degrees

# 参数说明
--x: 目标X坐标 (米)
--y: 目标Y坐标 (米)  
--yaw: 目标朝向角度
--degrees: 朝向单位为度数（否则为弧度）
--frame: 坐标系 (默认: odom)
```

### send_waypoints.py 功能
- 自动按顺序导航多个路径点
- 内置正方形路径示例
- 自动检测到达并切换下一个点
- 支持自定义路径点

## 🚫 限制说明

### ✅ 适用场景
- 平坦地面环境
- 无动态障碍物  
- 短距离导航 (< 10m)
- 里程计精度较好的机器人

### ⚠️ 不适用场景
- 需要避障功能
- 长距离导航（累积误差大）
- 复杂地形环境
- 需要SLAM建图

## 🔍 调试技巧

### 查看导航状态
```bash
# 查看里程计数据
ros2 topic echo /diff_drive_controller/odom

# 查看目标位置
ros2 topic echo /goal_pose

# 查看速度命令
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped
```

### 参数调优建议
- **机器人响应慢**: 增大 `linear_kp` 和 `angular_kp`
- **机器人振荡**: 减小 `angular_kp`
- **转向过快**: 减小 `max_angular_vel`
- **前进太慢**: 增大 `max_linear_vel`

## 📈 性能优化

### 提高定位精度
1. 确保轮子与地面良好接触
2. 定期校准轮径和轮距参数
3. 使用高精度编码器
4. 避免在光滑地面上使用

### 提高导航平滑度
1. 调整控制频率 (默认20Hz)
2. 优化PID参数
3. 设置合理的速度限制
4. 调整到达容差

## 🤝 扩展功能

这个包为基础导航功能，可以扩展：

- 添加避障功能
- 集成视觉或激光传感器
- 实现路径规划算法
- 添加动态重规划
- 支持多目标点队列

---

**开发者**: sirius  
**版本**: 1.0.0  
**许可证**: Apache-2.0
