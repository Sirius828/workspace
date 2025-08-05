# Jump Start - 简单的FSM机器人控制包

这是一个基于有限状态机（FSM）的简单机器人控制包，提供导航和目标跟踪功能。**现在完全兼容 start_engine 包，支持通过GUI启动！**

## 🚀 新功能：与 start_engine 包集成

`jump_start` 现在提供 `/start` 服务接口，完全兼容您的 `start_engine` GUI包：

- 🔧 **服务接口**: 提供 `/start` 服务 (std_srvs/Empty)
- 🖱️ **GUI触发**: 通过 start_engine GUI 的"启动系统"按钮触发
- 🎯 **默认任务**: 自动执行搜索和跟踪任务
- 📍 **智能路径**: 4x4地图优化的搜索路径

### 快速启动集成系统
```bash
# 启动完整的集成系统（FSM + GUI）
ros2 launch jump_start integrated_system.launch.py

# 或者分别启动
# 终端1: FSM控制器
ros2 run jump_start fsm_controller

# 终端2: Start Engine GUI
ros2 run start_engine start_gui
```

### 使用方法
1. 启动集成系统
2. 在GUI中点击 **"启动系统"** 按钮
3. FSM自动开始执行搜索和跟踪任务
4. 观察机器人按预设路径搜索目标

## 功能特性

### FSM状态
- **IDLE**: 空闲状态，等待任务指令
- **NAVIGATING**: 导航到指定位置
- **TARGET_TRACKING**: 目标跟踪模式
- **VICTORY**: 任务成功完成
- **ERROR**: 错误状态
- **EMERGENCY_STOP**: 紧急停止

### 主要功能
1. **位置导航**: 控制机器人移动到指定的x,y,z坐标
2. **目标跟踪**: 监控视觉目标，当目标在图像中心停留足够时间时触发胜利
3. **异常处理**: 支持紧急停止和错误状态处理
4. **状态监控**: 实时发布FSM状态和系统状态信息

## 话题接口

### 订阅的话题
- `/chassis/odom` (nav_msgs/Odometry): 机器人里程计信息
- `/target_position_pixel` (geometry_msgs/Point): 目标在图像中的像素位置
- `/mission_command` (std_msgs/String): 任务命令
- `/emergency_stop` (std_msgs/Bool): 紧急停止信号

### 发布的话题
- `/target_pose_with_speed` (geometry_msgs/TwistStamped): 目标位置和速度命令
- `/victory` (std_msgs/Bool): 胜利状态
- `/fsm_state` (std_msgs/String): 当前FSM状态
- `/fsm_status` (std_msgs/String): 详细状态信息

### 服务接口
- `/start` (std_srvs/Empty): **启动默认任务**（与 start_engine 包兼容）

## 使用方法

### 🚀 推荐：集成系统启动（含GUI）
```bash
# 启动完整系统（FSM控制器 + start_engine GUI）
ros2 launch jump_start integrated_system.launch.py

# 可选参数
ros2 launch jump_start integrated_system.launch.py enable_gui:=true log_level:=info
```

### 1. 单独启动FSM控制器
```bash
# 启动FSM控制器和任务执行器
ros2 launch jump_start jump_start.launch.py

# 或者单独启动FSM控制器
ros2 run jump_start fsm_controller
```

### 2. 通过 start_engine GUI 启动任务
```bash
# 如果已启动FSM控制器，再启动GUI
ros2 run start_engine start_gui

# 然后在GUI中点击"启动系统"按钮
```

### 3. 通过服务调用启动任务
```bash
# 直接调用 /start 服务
ros2 service call /start std_srvs/srv/Empty
```

### 2. 发送任务命令

#### 导航命令
```bash
# 导航到位置 (x=1.0, y=1.0, z=0.3)
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'navigate 1.0 1.0 0.3'"
```

#### 目标跟踪命令
```bash
# 开始目标跟踪
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'track'"
```

#### 控制命令
```bash
# 停止当前任务
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'stop'"

# 重置系统
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'reset'"
```

### 3. 使用任务执行器（交互模式）
```bash
ros2 run jump_start mission_executor
```

### 4. 监控系统状态
```bash
# 监控FSM状态
ros2 topic echo /fsm_state

# 监控详细状态
ros2 topic echo /fsm_status

# 监控胜利状态
ros2 topic echo /victory
```

## 配置参数

在 `config/fsm_config.yaml` 中可以配置以下参数：

- `position_tolerance`: 位置到达容差 (默认: 0.3米)
- `pixel_tolerance`: 像素位置容差 (默认: 50像素)
- `victory_time_threshold`: 胜利时间阈值 (默认: 2.0秒)
- `image_width/height`: 图像尺寸 (默认: 640x480)
- `navigation_timeout`: 导航超时时间 (默认: 30秒)
- `valid_tracking_x`: 有效跟踪的x坐标阈值 (默认: 2.0米)
- `map_size`: 地图尺寸 (默认: 4.0米)

## 工作原理

### 导航控制
FSM控制器接收导航命令后：
1. 切换到NAVIGATING状态
2. 持续发布目标位置到 `/target_pose_with_speed`
3. 监控当前位置与目标位置的距离
4. 当距离小于容差时，切换到VICTORY状态

### 目标跟踪
FSM控制器在TARGET_TRACKING状态时：
1. 监听 `/target_position_pixel` 话题
2. **检查机器人是否在有效跟踪区域（x > 2.0米，即通过4x4地图的半场）**
3. **只有在有效区域内，目标在图像中心时才开始计时**
4. 如果目标在中心区域停留超过阈值时间且位置有效，发布胜利信号
5. 切换到VICTORY状态

### 位置约束逻辑
- **4x4米地图**：机器人工作在4x4米的场地内
- **有效跟踪区域**：只有当机器人x坐标 > 2.0米时（通过半场），云台跟踪才有效
- **计时重置**：如果机器人离开有效区域，计时器自动重置
- **智能提示**：系统会实时显示是否在有效跟踪区域内

### 控制命令格式
发送到 `/target_pose_with_speed` 的TwistStamped消息格式：
```yaml
header:
  frame_id: 'odom'
twist:
  linear: {x: 目标X, y: 目标Y, z: 目标Z}
  angular: {z: 目标角度, x: 0.0, y: 0.0}
```

## 与其他包的集成

### 与 yolo_detector 集成
- yolo_detector 发布 `/target_position_pixel`
- FSM控制器订阅此话题进行目标跟踪

### 与底盘控制集成
- FSM控制器发布 `/target_pose_with_speed`
- 底盘控制器订阅此话题执行运动控制
- 底盘发布 `/chassis/odom` 提供位置反馈

## 新增功能测试

### 位置跟踪测试
```bash
# 运行完整的位置跟踪测试（需要先启动FSM控制器）
cd /home/sirius/ssd/ros2workspace
source install/setup.bash

# 方法1: 自动化测试
python3 src/jump_start/position_tracking_test.py

# 方法2: 交互式演示
./src/jump_start/position_demo.sh
```

#### 自动化测试包括：
1. **无效区域测试**: 在x ≤ 2.0区域，目标跟踪不应该计时和触发胜利
2. **有效区域测试**: 在x > 2.0区域，目标跟踪正常工作
3. **区域切换测试**: 离开有效区域时计时器自动重置

#### 手动测试步骤：
```bash
# 终端1: 启动FSM控制器
ros2 run jump_start fsm_controller

# 终端2: 设置机器人在无效区域 (x=1.5)
ros2 topic pub --once /chassis/odom nav_msgs/msg/Odometry "{
  header: {frame_id: 'odom'},
  pose: {pose: {position: {x: 1.5, y: 2.0, z: 0.0}}}
}"

# 终端3: 开始跟踪
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'track'"

# 终端4: 发布目标在图像中心（应该不会胜利）
ros2 topic pub -r 10 /target_position_pixel geometry_msgs/msg/Point "x: 320.0, y: 240.0, z: 0.9"

# 终端5: 移动到有效区域 (x=2.5)
ros2 topic pub --once /chassis/odom nav_msgs/msg/Odometry "{
  header: {frame_id: 'odom'},
  pose: {pose: {position: {x: 2.5, y: 2.0, z: 0.0}}}
}"

# 现在目标跟踪应该正常工作，2秒后触发胜利
```

#### 验证要点：
- 在无效区域（x ≤ 2.0）时目标跟踪不计时
- 进入有效区域（x > 2.0）时开始正常计时
- 离开有效区域时计时器重置
- 位置约束对胜利条件的影响
- 系统状态正确显示当前区域状态

### 搜索和跟踪任务
```python
# 使用任务执行器进行搜索和跟踪
waypoints = [
    (1.0, 0.0, 0.3),
    (1.0, 1.0, 0.3),
    (0.0, 1.0, 0.3),
    (0.0, 0.0, 0.3),
]
```

1. 机器人依次访问各个路径点
2. 在每个点尝试检测和跟踪目标
3. 一旦目标跟踪成功，任务完成

## 扩展功能

FSM设计支持轻松添加新状态和功能：
- 添加新的状态枚举
- 实现对应的状态处理函数
- 添加状态转换逻辑
- 定义新的任务命令格式

## 故障排除

### 常见问题
1. **导航不响应**: 检查 `/chassis/odom` 话题是否发布
2. **目标跟踪不工作**: 检查 `/target_position_pixel` 话题是否发布
3. **状态卡住**: 使用reset命令重置系统

### 调试命令
```bash
# 检查话题列表
ros2 topic list

# 检查话题数据
ros2 topic echo /fsm_state
ros2 topic echo /target_position_pixel
ros2 topic echo /chassis/odom

# 检查节点状态
ros2 node list
ros2 node info /fsm_controller
```
