# 🤖 ROS2 机器人行为树系统

基于您现有功能包设计的完整机器人行为树系统，整合了底盘控制、云台追踪、目标检测和位置控制等功能。

## ✨ 功能特性

### 🎯 核心功能
- **智能目标追踪**: 基于YOLO检测的实时目标追踪
- **自主巡逻**: 多点路径巡逻，支持循环模式
- **云台控制**: 二轴云台精确控制和目标跟随
- **位置导航**: 精确位置控制和路径规划
- **行为决策**: 基于BehaviorTree的智能行为决策

### 🧠 行为树架构
```
机器人主行为
├── 系统初始化
├── 目标追踪任务 (优先级最高)
│   ├── 目标检测
│   ├── 云台调整  
│   └── 跟随移动
├── 巡逻任务
│   ├── 路径规划
│   ├── 云台扫描
│   └── 位置导航
└── 空闲状态
    ├── 停止运动
    └── 扫描模式
```

## 🚀 快速开始

### 1. 编译安装
```bash
cd /home/sirius/ssd/ros2workspace
colcon build --packages-select robot_behavior_tree
source install/setup.bash
```

### 2. 启动完整系统
```bash
# 启动完整的机器人行为树系统
ros2 launch robot_behavior_tree robot_behavior_tree.launch.py

# 仅启动行为树核心（不含硬件）
ros2 launch robot_behavior_tree robot_behavior_tree.launch.py enable_hardware:=false

# 自定义行为树文件
ros2 launch robot_behavior_tree robot_behavior_tree.launch.py \
    behavior_tree_file:=/path/to/your/behavior_tree.xml
```

### 3. 交互式监控
```bash
# 启动监控界面
ros2 run robot_behavior_tree behavior_tree_monitor.py

# 仅显示状态
ros2 run robot_behavior_tree behavior_tree_monitor.py --status-only
```

### 4. 任务演示
```bash
# 交互式演示菜单
ros2 run robot_behavior_tree mission_demo.py

# 自动演示序列
ros2 run robot_behavior_tree mission_demo.py --auto

# 单独演示功能
ros2 run robot_behavior_tree mission_demo.py --search   # 搜索追踪
ros2 run robot_behavior_tree mission_demo.py --patrol   # 巡逻任务
```

## 📋 系统组件

### 🔧 核心节点
- **behavior_tree_executor**: 行为树执行引擎
- **behavior_tree_monitor**: 监控和调试工具
- **mission_planner**: 任务规划器

### 🎮 控制接口
```bash
# 启动/停止行为树
ros2 service call /behavior_tree/control std_srvs/srv/SetBool "{data: true}"

# 发送命令
ros2 topic pub /behavior_tree/command std_msgs/msg/String "{data: 'start'}"
ros2 topic pub /behavior_tree/command std_msgs/msg/String "{data: 'stop'}"
ros2 topic pub /behavior_tree/command std_msgs/msg/String "{data: 'reset'}"

# 监控状态
ros2 topic echo /behavior_tree/status
```

## 🌳 行为树节点说明

### 🚶 导航节点
- `NavigateToPosition`: 导航到指定位置
- `GetCurrentPosition`: 获取当前位置
- `IsAtPosition`: 检查是否到达位置
- `StopRobot`: 停止机器人
- `RotateToAngle`: 旋转到指定角度

### 📷 云台节点
- `GimbalRotateTo`: 云台转向指定角度
- `EnableGimbalTracking`: 启用/禁用目标追踪
- `GimbalCenter`: 云台回中
- `IsGimbalTracking`: 检查追踪状态
- `GimbalScanMode`: 云台扫描模式

### 🎯 检测节点
- `IsTargetDetected`: 检查目标检测
- `WaitForTarget`: 等待目标出现
- `StartDetection`: 启动检测
- `StopDetection`: 停止检测
- `IsTargetCentered`: 检查目标是否居中

## 📁 文件结构

```
robot_behavior_tree/
├── action/                     # Action定义
│   ├── RobotMission.action
│   ├── SearchAndTrack.action
│   └── PatrolMission.action
├── behavior_trees/             # 行为树XML文件
│   ├── robot_main_behavior.xml
│   └── search_and_track.xml
├── config/                     # 配置文件
│   └── behavior_tree_config.yaml
├── include/                    # C++头文件
│   └── robot_behavior_tree/
├── launch/                     # 启动文件
│   └── robot_behavior_tree.launch.py
├── scripts/                    # Python脚本
│   ├── behavior_tree_monitor.py
│   └── mission_demo.py
└── src/                        # C++源文件
    └── behavior_tree_executor.cpp
```

## 🎛️ 配置说明

### 主配置文件: `config/behavior_tree_config.yaml`
- **执行频率**: 行为树tick频率
- **导航参数**: 速度、容差等
- **云台参数**: 角度限制、扫描参数
- **检测参数**: 目标类别、置信度
- **安全参数**: 紧急停止、超时保护

### 行为树文件
- `robot_main_behavior.xml`: 主行为树（追踪+巡逻）
- `search_and_track.xml`: 专用搜索追踪行为树

## 🔄 与现有系统集成

### 话题接口
```bash
# 输入话题
/cmd_vel                    # 速度控制
/chassis/odom              # 里程计数据
/target_position_pixel     # 目标像素位置
/cmd_gimbal               # 云台控制

# 输出话题
/behavior_tree/status     # 行为树状态
/behavior_tree/command    # 行为树命令
```

### 功能包依赖
- `chassis_hardware`: 底盘硬件控制
- `chassis_position_controller`: 位置控制
- `gimbal_pixel_controller`: 云台像素控制
- `yolo_detector`: 目标检测

## 🎯 任务场景

### 1. 目标追踪模式
```xml
<!-- 检测到目标后自动追踪 -->
<ReactiveSequence>
    <Condition ID="IsTargetDetected"/>
    <Action ID="EnableGimbalTracking"/>
    <SubTree ID="FollowTarget"/>
</ReactiveSequence>
```

### 2. 巡逻模式
```xml
<!-- 未检测到目标时执行巡逻 -->
<ReactiveSequence>
    <Inverter>
        <Condition ID="IsTargetDetected"/>
    </Inverter>
    <SubTree ID="ExecutePatrolPath"/>
</ReactiveSequence>
```

### 3. 搜索模式
```xml
<!-- 主动搜索目标 -->
<Sequence>
    <Action ID="GimbalScanMode"/>
    <Action ID="WaitForTarget"/>
    <SubTree ID="TrackTarget"/>
</Sequence>
```

## 🛠️ 调试和监控

### 实时状态监控
```bash
# 启动监控界面
ros2 run robot_behavior_tree behavior_tree_monitor.py
```

### 行为树可视化
- 支持BehaviorTree日志记录
- 生成执行轨迹文件
- 实时状态输出

### 性能监控
- 执行频率监控
- 节点性能统计
- 系统资源使用

## 🔧 扩展开发

### 添加新的行为树节点
1. 在相应的头文件中定义节点类
2. 实现节点逻辑
3. 在factory中注册节点
4. 在XML中使用节点

### 自定义任务
1. 创建新的Action定义
2. 实现Action服务器
3. 设计对应的行为树
4. 添加到启动配置

## ⚠️ 注意事项

1. **安全第一**: 确保在安全环境中测试
2. **参数调优**: 根据实际机器人调整速度和容差参数
3. **硬件兼容**: 确认所有硬件节点正常运行
4. **网络延迟**: 注意ROS2通信延迟对实时性的影响

## 🤝 支持和贡献

如有问题或建议，请通过以下方式联系：
- 创建Issue描述问题
- 提交Pull Request改进代码
- 参与讨论分享经验

## 📜 许可证

本项目基于Apache-2.0许可证开源。
