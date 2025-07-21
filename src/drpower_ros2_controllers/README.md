# DrPower ROS2 Controllers 说明书

## 📖 概述

DrPower ROS2 Controllers 是为大然机器人（DrPower）智能一体化关节专门开发的ROS2硬件接口包。它基于原始的Python CAN通信协议，提供了完整的C++实现，支持ros2_control框架和MoveIt2集成。

## 🎯 主要特性

- ✅ **完整的CAN通信协议**：基于DrEmpower.py实现，保持100%兼容性
- ✅ **多控制接口**：支持位置、速度、力矩控制
- ✅ **灵活的电机组合**：支持6轴机械臂、2轴云台、单轴夹爪等任意组合
- ✅ **同步运动控制**：多电机协调运动，确保轨迹同步
- ✅ **实时状态反馈**：高频率状态更新，支持闭环控制
- ✅ **安全保护**：急停、限位、错误检测和恢复
- ✅ **标准化接口**：完全符合ros2_control规范，支持MoveIt2

## 🏗️ 系统架构

```
┌─────────────────┐    ┌────────────────────┐    ┌──────────────────┐
│    MoveIt2      │    │   ros2_control     │    │   DrPower Motors │
│                 │◄──►│                    │◄──►│                  │
│ - 轨迹规划      │    │ - 硬件抽象层       │    │ - CAN总线通信    │
│ - 运动学解算    │    │ - 控制器管理       │    │ - 实时状态反馈   │
│ - 碰撞检测      │    │ - 安全监控         │    │ - 多模式控制     │
└─────────────────┘    └────────────────────┘    └──────────────────┘
```

## 🔧 底层工作原理

### CAN通信协议

基于原始DrEmpower.py的通信格式：

#### 1. CAN ID计算
```cpp
uint16_t can_id = (motor_id << 5) + command_id;
```

#### 2. 数据帧格式
```
┌──────┬──────┬──────┬─────────────────────────────────┐
│ DLC  │ ID_H │ ID_L │         Data (8 bytes)          │
│ 0x08 │      │      │                                 │
└──────┴──────┴──────┴─────────────────────────────────┘
```

#### 3. 主要命令字
- `0x08`: 通用控制命令（包含启动命令）
- `0x0C`: 预设位置命令
- `0x19`: 位置控制（轨迹跟踪）
- `0x1A`: 位置控制（梯形轨迹）
- `0x1B`: 位置控制（前馈控制）
- `0x1C`: 速度控制
- `0x1D`: 力矩控制

#### 4. 启动命令（多电机同步）
- `0x10`: 轨迹跟踪模式启动
- `0x11`: 梯形轨迹模式启动
- `0x12`: 前馈控制模式启动
- `0x13`: 速度控制模式启动

### 数据格式转换

支持多种数据类型的自动转换：

| 格式 | 字节数 | 说明 | 用途 |
|------|--------|------|------|
| `f` | 4 | float | 角度、速度、力矩值 |
| `s16` | 2 | signed int16 | 时间、加速度参数 |
| `u16` | 2 | unsigned int16 | 模式、状态标志 |
| `s32` | 4 | signed int32 | 扩展参数 |
| `u32` | 4 | unsigned int32 | 启动命令 |

### 多电机协调机制

#### 同步控制流程
1. **预设阶段**：向每个电机发送目标参数
   ```cpp
   can_driver_->preset_position_command(motor_id, position, velocity, param, mode);
   ```

2. **启动阶段**：广播同步启动命令
   ```cpp
   can_driver_->send_start_command(start_command);
   ```

#### 独立控制流程
直接发送控制命令，立即执行：
```cpp
can_driver_->send_position_command(motor_id, position, velocity, param, mode);
```

## 📋 接口和参数

### 硬件接口参数

#### 基本配置
```yaml
drpower_hardware:
  ros__parameters:
    device_path: "/dev/ttyUSB0"      # CAN设备路径
    baudrate: 115200                 # 通信波特率
    control_frequency: 100.0         # 控制频率(Hz)
    enable_state_feedback: true      # 开启状态反馈
    state_feedback_rate_ms: 10       # 反馈间隔(ms)
```

#### 电机配置
```yaml
# URDF中的关节配置
<joint name="joint1">
  <param name="id">1</param>                        # 电机ID (1-64)
  <param name="position_mode">1</param>              # 位置控制模式
  <param name="position_acceleration">15.0</param>   # 位置控制加速度
  <param name="velocity_mode">1</param>              # 速度控制模式
  <param name="velocity_acceleration">10.0</param>   # 速度控制加速度
</joint>
```

### 控制模式详解

#### 位置控制模式
- **Mode 0 - 轨迹跟踪**：适合实时轨迹跟随
  - `param`: 滤波带宽 (<300)
  - `speed`: 前馈速度 (r/min)
  
- **Mode 1 - 梯形轨迹**：适合点到点运动
  - `param`: 加速度 (r/min/s)
  - `speed`: 目标速度 (r/min)
  
- **Mode 2 - 前馈控制**：适合高精度控制
  - `param`: 前馈力矩 (Nm)
  - `speed`: 前馈速度 (r/min)

#### 速度控制模式
- **Mode 0 - 直接控制**：直接设置目标速度
  - `param`: 前馈力矩 (Nm)
  
- **Mode 1 - 匀加速控制**：平滑加速到目标速度
  - `param`: 角加速度 (r/min/s)

### 状态接口

每个关节提供以下状态信息：

```cpp
// 基本状态
state_interfaces:
  - position    // 当前位置 (弧度)
  - velocity    // 当前速度 (弧度/秒)
  - effort      // 当前力矩 (Nm)

// 扩展状态（通过MotorState结构访问）
struct MotorState {
  double voltage;           // 总线电压 (V)
  double current;           // 电流 (A)
  double temperature;       // 温度 (°C)
  bool online;              // 在线状态
  bool enabled;             // 使能状态
  uint32_t error_code;      // 错误代码
  // ... 更多状态信息
};
```

### 命令接口

每个关节支持以下命令：

```cpp
command_interfaces:
  - position    // 位置命令 (弧度)
  - velocity    // 速度命令 (弧度/秒)  
  - effort      // 力矩命令 (Nm)
```

## 🚀 使用方法

### 1. 环境准备

```bash
# 确保ROS2环境已设置
source /opt/ros/humble/setup.bash
cd ~/ros2_workspace

# 安装依赖
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-joint-trajectory-controller
sudo apt install ros-humble-robot-state-publisher
```

### 2. 编译安装

```bash
# 编译包
colcon build --packages-select drpower_hardware_interface

# 设置环境
source install/setup.bash
```

### 3. 硬件连接

1. 连接CAN转USB模块到计算机
2. 确认设备路径：`ls /dev/ttyUSB*`
3. 设置串口权限：`sudo chmod 666 /dev/ttyUSB0`

### 4. 启动系统

#### 六轴机械臂
```bash
ros2 launch drpower_hardware_interface drpower_arm.launch.py \
  device_path:=/dev/ttyUSB0 \
  baudrate:=115200
```

#### 二轴云台
```bash
ros2 launch drpower_hardware_interface drpower_arm.launch.py \
  description_file:=drpower_gimbal.urdf.xacro \
  controllers_file:=drpower_gimbal_controllers.yaml
```

### 5. 控制示例

#### 通过命令行控制
```bash
# 发送关节轨迹命令
ros2 action send_goal /arm_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
      points: [{
        positions: [0.0, -1.57, 1.57, 0.0, 1.57, 0.0],
        time_from_start: {sec: 5}
      }]
    }
  }"
```

#### 通过Python API控制
```python
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient

class DrPowerController(Node):
    def __init__(self):
        super().__init__('drpower_controller')
        self.action_client = ActionClient(
            self, FollowJointTrajectory, 
            '/arm_joint_trajectory_controller/follow_joint_trajectory'
        )
    
    def send_trajectory(self, positions, duration=5.0):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        goal_msg.trajectory.points = [point]
        
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal_msg)

# 使用示例
def main():
    rclpy.init()
    controller = DrPowerController()
    
    # 发送到目标位置
    future = controller.send_trajectory([0.0, -1.57, 1.57, 0.0, 1.57, 0.0])
    rclpy.spin_until_future_complete(controller, future)
    
    rclpy.shutdown()
```

### 6. 与MoveIt2集成

```python
import rclpy
from moveit import MoveItPy

def main():
    rclpy.init()
    
    # 初始化MoveIt
    robot = MoveItPy(node_name="moveit_py")
    arm = robot.get_planning_component("arm")
    
    # 设置目标位置
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="home")
    
    # 规划轨迹
    plan_result = arm.plan()
    
    if plan_result:
        # 执行轨迹
        arm.execute(plan_result.trajectory)
    
    rclpy.shutdown()
```

## 🔍 调试和监控

### 1. 查看控制器状态
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### 2. 监控关节状态
```bash
ros2 topic echo /joint_states
ros2 topic echo /dynamic_joint_states
```

### 3. 监控硬件状态
```bash
# 查看硬件接口日志
ros2 log set_logger_level controller_manager.drpower_hardware DEBUG

# 监控CAN通信
sudo candump can0  # 如果使用SocketCAN
```

### 4. 错误诊断

#### 常见问题和解决方案

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 无法连接设备 | 设备路径错误/权限不足 | 检查`/dev/ttyUSB*`，设置权限 |
| 电机无响应 | ID配置错误/通信中断 | 检查电机ID，验证CAN连接 |
| 轨迹执行不平滑 | 控制频率过低/参数不当 | 调整`control_frequency`和加速度参数 |
| 多电机不同步 | 同步模式未启用 | 设置`synchronized: true` |

## 📊 性能参数

### 通信性能
- **通信频率**：最高460800 baud
- **控制频率**：推荐100Hz，最高1000Hz
- **状态反馈**：可配置1-1000ms间隔
- **延迟**：<5ms（100Hz控制频率）

### 精度指标
- **位置精度**：±0.01°（取决于电机规格）
- **速度精度**：±1 r/min
- **力矩精度**：±0.01 Nm

### 系统限制
- **最大电机数**：64个（受CAN ID限制）
- **最大控制组数**：16个
- **同时控制电机数**：建议≤8个（性能考虑）

## 🛡️ 安全特性

### 1. 硬件安全
- **急停功能**：软件急停 + 硬件急停
- **限位保护**：软件限位 + 硬件限位
- **过载保护**：电流、温度、电压监控

### 2. 软件安全
- **通信超时检测**：自动断开失联电机
- **状态监控**：实时错误检测和报告
- **轨迹验证**：执行前验证轨迹安全性

### 3. 紧急处理
```cpp
// 紧急停止所有电机
emergency_stop();

// 单独停止某个电机
can_driver_->emergency_stop(motor_id);
```

## 🧪 测试与验证

### 单元测试
```bash
# 编译并运行单元测试
cd /path/to/ros2workspace
colcon build --packages-select drpower_hardware_interface
colcon test --packages-select drpower_hardware_interface

# 查看测试结果
colcon test-result --verbose
```

### 集成测试
```bash
# 1. 基础功能测试
source install/setup.bash
ros2 launch drpower_hardware_interface drpower_arm.launch.py

# 2. 运行机械臂测试脚本
python3 src/drpower_ros2_controllers/drpower_hardware_interface/examples/test_drpower_arm.py

# 3. 运行云台测试脚本  
python3 src/drpower_ros2_controllers/drpower_hardware_interface/examples/test_drpower_gimbal.py

# 4. 运行高级综合测试
python3 src/drpower_ros2_controllers/drpower_hardware_interface/examples/advanced_test.py
```

### 快速测试
```bash
# 使用一键测试脚本
cd /path/to/ros2workspace/src/drpower_ros2_controllers
./quick_test.sh
```

### MoveIt2集成测试
```bash
# 启动MoveIt2演示
source install/setup.bash
ros2 launch drpower_moveit_config drpower_moveit_demo.launch.py

# 使用MoveIt2规划和执行运动
# (在RViz中通过MoveIt Motion Planning插件)
```

## 🔌 MoveIt2集成

### MoveIt2配置包
本项目包含完整的MoveIt2配置包 `drpower_moveit_config`，支持：

- ✅ **运动规划**：使用OMPL规划器进行路径规划
- ✅ **逆运动学求解**：基于KDL的运动学求解
- ✅ **碰撞检测**：实时碰撞检测和避障
- ✅ **可视化界面**：RViz2集成的交互式规划界面
- ✅ **轨迹优化**：时间最优化轨迹生成

### 启动MoveIt2
```bash
# 启动完整的MoveIt2演示
ros2 launch drpower_moveit_config drpower_moveit_demo.launch.py

# 仅启动move_group服务器
ros2 launch drpower_moveit_config move_group.launch.py

# 仅启动RViz界面
ros2 launch drpower_moveit_config moveit_rviz.launch.py
```

### MoveIt2 Python API示例
```python
import rclpy
from moveit_interface import MoveGroupInterface

def moveit_example():
    rclpy.init()
    node = rclpy.create_node('moveit_example')
    
    # 创建move group接口
    arm = MoveGroupInterface("drpower_arm", "base_link", node)
    
    # 设置目标位置
    arm.set_pose_target([0.3, 0.2, 0.5, 0.0, 0.0, 0.0, 1.0])
    
    # 规划并执行
    plan = arm.plan()
    if plan:
        arm.execute(plan)
        
    rclpy.shutdown()
```

## 🔧 高级配置

### 自定义电机组

```yaml
motor_groups:
  custom_group:
    motor_ids: [1, 3, 5]           # 自定义电机组合
    joint_names: ["joint1", "joint3", "joint5"]
    synchronized: true              # 同步控制
    position_control:
      mode: 2                       # 前馈控制
      feedforward_velocity: 10.0
      feedforward_torque: 0.5
```

### 动态参数调整

```bash
# 运行时修改参数
ros2 param set /controller_manager position_mode 2
ros2 param set /controller_manager position_acceleration 20.0
```

### 自定义控制器

继承`hardware_interface::SystemInterface`创建专用控制器：

```cpp
class CustomDrPowerController : public DrPowerHardwareInterface {
public:
  // 重写控制逻辑
  hardware_interface::return_type write(
    const rclcpp::Time & time, 
    const rclcpp::Duration & period) override;
};
```

## 📚 扩展开发

### 添加新的控制模式

1. 在`MotorControlModes`中定义新模式
2. 在`CanDriver`中实现通信协议
3. 在`DrPowerHardwareInterface`中添加控制逻辑

### 集成外部传感器

```cpp
class ExtendedDrPowerInterface : public DrPowerHardwareInterface {
private:
  std::unique_ptr<ExternalSensor> sensor_;
  
public:
  hardware_interface::return_type read(
    const rclcpp::Time & time, 
    const rclcpp::Duration & period) override {
    // 读取电机状态
    DrPowerHardwareInterface::read(time, period);
    
    // 读取外部传感器
    sensor_->read_data();
    return hardware_interface::return_type::OK;
  }
};
```

## 📖 技术支持

### 文档和资源
- **官方文档**：[ROS2 Control Documentation](https://control.ros.org/)
- **示例代码**：`examples/` 目录
- **API参考**：Doxygen生成的API文档

### 社区支持
- **Issues**：GitHub Issues页面
- **讨论**：ROS Discourse论坛
- **贡献**：欢迎提交PR和功能请求

---

## 📝 总结

DrPower ROS2 Controllers提供了完整的硬件抽象层，让您能够：

1. **获得底层完全控制权**：直接访问CAN通信协议
2. **享受标准化接口**：兼容ros2_control生态系统
3. **灵活配置电机组合**：支持机械臂、云台、夹爪等多种应用
4. **实现高性能控制**：100Hz控制频率，<5ms延迟
5. **确保系统安全**：多层安全保护机制

通过这个接口，您可以轻松地将DrPower电机集成到复杂的机器人系统中，同时保持代码的可维护性和可扩展性。
