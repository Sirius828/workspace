# 键盘遥控包 (keyboard_teleop)

这个包提供键盘控制机器人运动的功能，支持步进控制和速度叠加。

## 功能特性

### 🎮 控制方式
- **步进控制**: 每次按键增加/减少固定步长的速度
- **速度叠加**: 反复按同一方向键可以叠加速度
- **实时反馈**: 显示当前速度状态
- **安全保护**: 自动超时停止、最大速度限制

### ⌨️ 按键映射
```
移动控制:
  w/s: 前进/后退  (x方向线速度)
  a/d: 左转/右转  (角速度)
  q/e: 左移/右移  (y方向线速度)

功能控制:
  空格: 立即停止所有运动
  r: 重置所有速度为0
  +/-: 增加/减少步进大小
  h: 显示帮助信息
  Ctrl+C: 退出程序
```

## 节点说明

### keyboard_teleop_node (完整版)
功能丰富的键盘遥控节点:
- 可配置参数
- 步进大小动态调整
- 超时自动停止
- 详细状态显示

### simple_keyboard_teleop_node (简化版)
轻量级键盘遥控节点:
- 固定参数，快速启动
- 简洁界面
- 适合快速测试

## 参数配置

### 控制参数
- `step_size`: 每次按键的速度步进 (默认: 0.05)
- `max_linear_vel`: 最大线速度 (默认: 2.0 m/s)
- `max_angular_vel`: 最大角速度 (默认: 2.0 rad/s)

### 发布配置
- `cmd_vel_topic`: 发布的话题名称 (默认: "/cmd_vel")
- `publish_rate`: 发布频率 (默认: 10.0 Hz)
- `auto_stop_timeout`: 自动停止超时时间 (默认: 1.0秒)

## 使用方法

### 1. 构建包
```bash
cd /home/sirius/ssd/ros2workspace
colcon build --packages-select keyboard_teleop
source install/setup.bash
```

### 2. 启动完整版键盘遥控
```bash
ros2 launch keyboard_teleop keyboard_teleop.launch.xml
```

### 3. 启动简化版键盘遥控
```bash
ros2 launch keyboard_teleop simple_keyboard_teleop.launch.xml
```

### 4. 直接运行节点
```bash
# 完整版
ros2 run keyboard_teleop keyboard_teleop_node

# 简化版
ros2 run keyboard_teleop simple_keyboard_teleop_node
```

## 使用示例

### 基本操作
1. 启动节点后，按 `w` 键向前移动
2. 再次按 `w` 键，速度会增加 0.05 m/s
3. 继续按 `w` 键可以持续加速到最大速度
4. 按 `s` 键减速或反向
5. 按空格键立即停止

### 调整步进
1. 按 `+` 键增加步进大小
2. 按 `-` 键减少步进大小
3. 步进范围: 0.01 - 0.5

### 复合运动
可以同时控制多个方向:
- 同时按 `w` 和 `a`: 向前并左转
- 同时按 `w` 和 `q`: 向前并左移

## 速度叠加示例

```
初始状态: vx=0.0, vy=0.0, omega=0.0
按 w: vx=0.05, vy=0.0, omega=0.0
按 w: vx=0.10, vy=0.0, omega=0.0  ← 速度叠加
按 a: vx=0.10, vy=0.0, omega=0.05
按 a: vx=0.10, vy=0.0, omega=0.10 ← 角速度叠加
按空格: vx=0.0, vy=0.0, omega=0.0  ← 立即停止
```

## 安全特性

1. **最大速度限制**: 防止速度超过安全范围
2. **超时自动停止**: 1秒无按键自动停止
3. **立即停止**: 空格键紧急停止
4. **优雅退出**: Ctrl+C时发送停止命令

## 与其他包集成

### 配合轮式里程计使用
```bash
# 终端1: 启动里程计和串口通信
ros2 launch wheel_odom_driver wheel_odom_bidirectional.launch.xml

# 终端2: 启动键盘遥控
ros2 launch keyboard_teleop keyboard_teleop.launch.xml
```

### 配合导航使用
```bash
# 可以通过键盘控制在导航过程中手动控制机器人
ros2 launch keyboard_teleop simple_keyboard_teleop.launch.xml
```

## 故障排除

### 按键无响应
1. 确保终端窗口处于焦点状态
2. 检查是否有其他程序占用键盘输入
3. 尝试重启节点

### 速度不变
1. 检查 `/cmd_vel` 话题是否有其他发布者
2. 使用 `ros2 topic echo /cmd_vel` 验证消息发布
3. 检查机器人控制节点是否正常运行

### 程序退出异常
1. 使用 Ctrl+C 正常退出
2. 如果卡死，使用 `killall -9 python3`

## 注意事项

1. **终端焦点**: 使用时确保终端窗口处于焦点状态
2. **同时运行**: 避免同时运行多个键盘遥控节点
3. **安全距离**: 在有障碍物的环境中注意安全距离
4. **步进设置**: 根据机器人特性调整合适的步进大小
