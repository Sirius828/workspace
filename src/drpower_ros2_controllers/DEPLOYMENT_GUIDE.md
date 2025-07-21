# DrPower ROS2 Controllers 部署指南

## 🚀 快速开始

### 1. 系统要求
- **操作系统**: Ubuntu 22.04 LTS
- **ROS版本**: ROS2 Humble 
- **硬件**: DrPower智能一体化关节
- **接口**: CAN转USB模块 (如USB-CAN-II)

### 2. 安装步骤

#### 2.1 安装ROS2 Humble
```bash
# 添加ROS2仓库
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# 安装ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 安装开发工具
sudo apt install python3-colcon-common-extensions python3-rosdep
```

#### 2.2 安装依赖包
```bash
# ROS2 Control相关
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager ros-humble-hardware-interface

# MoveIt2相关 (可选)
sudo apt install ros-humble-moveit ros-humble-moveit-planners
sudo apt install ros-humble-moveit-servo ros-humble-moveit-visual-tools

# 其他依赖
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-state-publisher
```

#### 2.3 克隆和编译项目
```bash
# 创建工作空间
mkdir -p ~/drpower_ws/src
cd ~/drpower_ws/src

# 克隆项目（假设已经在ROS2工作空间中）
# 或直接复制drpower_ros2_controllers文件夹到src目录

# 安装rosdep依赖
cd ~/drpower_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译项目
source /opt/ros/humble/setup.bash
colcon build --packages-select drpower_hardware_interface drpower_moveit_config

# 设置环境
source install/setup.bash
```

### 3. 硬件连接

#### 3.1 CAN转USB连接
```bash
# 1. 连接CAN转USB模块到计算机
# 2. 查看设备
ls /dev/ttyUSB*   # 通常是 /dev/ttyUSB0

# 3. 设置设备权限
sudo chmod 666 /dev/ttyUSB0

# 4. 或者将用户添加到dialout组（推荐）
sudo usermod -a -G dialout $USER
# 注销重新登录生效
```

#### 3.2 电机ID配置
```bash
# 确保每个电机有唯一的ID (1-64)
# 默认配置假设机械臂使用ID: 1,2,3,4,5,6
# 云台使用ID: 1,2
```

### 4. 启动系统

#### 4.1 基础六轴机械臂
```bash
# 启动硬件接口和控制器
ros2 launch drpower_hardware_interface drpower_arm.launch.py \
  device_path:=/dev/ttyUSB0 \
  baudrate:=115200
```

#### 4.2 二轴云台
```bash
# 需要创建云台专用的配置文件
ros2 launch drpower_hardware_interface drpower_arm.launch.py \
  description_file:=drpower_gimbal.urdf.xacro \
  controllers_file:=drpower_gimbal_controllers.yaml
```

#### 4.3 MoveIt2集成
```bash
# 启动包含MoveIt2的完整系统
ros2 launch drpower_moveit_config drpower_moveit_demo.launch.py

# 在RViz中使用MoveIt Motion Planning插件进行交互式规划
```

### 5. 验证安装

#### 5.1 检查控制器状态
```bash
# 查看可用控制器
ros2 control list_controllers

# 查看硬件接口
ros2 control list_hardware_interfaces

# 查看关节状态
ros2 topic echo /joint_states
```

#### 5.2 运行测试脚本
```bash
# 机械臂测试
python3 src/drpower_ros2_controllers/drpower_hardware_interface/examples/test_drpower_arm.py

# 云台测试  
python3 src/drpower_ros2_controllers/drpower_hardware_interface/examples/test_drpower_gimbal.py

# 高级测试
python3 src/drpower_ros2_controllers/drpower_hardware_interface/examples/advanced_test.py
```

#### 5.3 一键测试
```bash
cd ~/drpower_ws/src/drpower_ros2_controllers
./quick_test.sh
```

## 🔧 配置自定义

### 1. 修改电机配置

编辑 `config/drpower_hardware.yaml`:
```yaml
drpower_hardware:
  ros__parameters:
    device_path: "/dev/ttyUSB0"      # 修改为你的设备路径
    baudrate: 115200                 # 修改波特率
    motor_ids: "1,2,3,4,5,6"        # 修改电机ID列表
    control_frequency: 100.0         # 调整控制频率
```

### 2. 自定义URDF

创建你的机器人描述文件:
```xml
<!-- your_robot.urdf.xacro -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="your_robot">
  
  <!-- 包含DrPower硬件接口 -->
  <xacro:include filename="$(find drpower_hardware_interface)/urdf/drpower_ros2_control.xacro"/>
  
  <!-- 使用DrPower硬件接口 -->
  <xacro:drpower_ros2_control 
    name="your_robot_hardware"
    device_path="/dev/ttyUSB0"
    motor_ids="1,2,3,4"
    joint_names="joint1,joint2,joint3,joint4"/>
    
  <!-- 你的机器人模型 -->
  <!-- ... -->
  
</robot>
```

### 3. 自定义控制器

创建控制器配置 `your_controllers.yaml`:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    
    your_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      
your_arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

## 🐛 常见问题解决

### 1. 设备连接问题

**问题**: `Failed to open device /dev/ttyUSB0`
```bash
# 检查设备是否存在
ls /dev/ttyUSB*

# 检查权限
ls -l /dev/ttyUSB0

# 设置权限
sudo chmod 666 /dev/ttyUSB0

# 或添加到用户组
sudo usermod -a -G dialout $USER
```

**问题**: `Device busy`
```bash
# 查看占用进程
sudo lsof /dev/ttyUSB0

# 杀死占用进程
sudo kill -9 <PID>
```

### 2. 电机无响应

**问题**: 电机ID配置错误
- 检查电机实际ID设置
- 确认配置文件中的ID匹配
- 使用示例脚本测试单个电机

**问题**: 通信参数错误
- 检查波特率设置
- 确认CAN转USB模块工作正常
- 测试串口通信

### 3. 控制器问题

**问题**: 控制器启动失败
```bash
# 查看控制器状态
ros2 control list_controllers

# 重新加载控制器
ros2 control reload_controller_libraries

# 检查配置文件语法
ros2 param describe /controller_manager
```

**问题**: 轨迹执行不平滑
- 降低控制频率 (50-100Hz)
- 调整轨迹时间参数
- 检查电机加速度设置

### 4. MoveIt2问题

**问题**: 运动学求解失败
- 检查URDF模型正确性
- 确认关节限制合理
- 调整规划器参数

**问题**: RViz显示异常
- 重启RViz
- 检查tf树完整性
- 确认话题发布正常

## 📚 进阶使用

### 1. 集成到现有项目

在你的package.xml中添加依赖:
```xml
<exec_depend>drpower_hardware_interface</exec_depend>
<exec_depend>drpower_moveit_config</exec_depend>
```

在你的launch文件中包含:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    drpower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('drpower_hardware_interface'), 
            '/launch/drpower_arm.launch.py'
        ])
    )
    
    return LaunchDescription([
        drpower_launch,
        # 你的其他节点...
    ])
```

### 2. 自定义控制逻辑

继承DrPowerHardwareInterface:
```cpp
#include "drpower_hardware_interface/drpower_hardware_interface.hpp"

class CustomDrPowerInterface : public drpower_hardware_interface::DrPowerHardwareInterface
{
public:
  hardware_interface::return_type write(
    const rclcpp::Time & time, 
    const rclcpp::Duration & period) override
  {
    // 自定义控制逻辑
    
    // 调用基类实现
    return DrPowerHardwareInterface::write(time, period);
  }
};
```

### 3. 性能监控

添加性能监控脚本:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.callback, 10)
        self.last_time = time.time()
        self.count = 0
        
    def callback(self, msg):
        current_time = time.time()
        self.count += 1
        
        if self.count % 100 == 0:  # 每100次打印一次
            freq = 100.0 / (current_time - self.last_time)
            self.get_logger().info(f'Joint state frequency: {freq:.1f} Hz')
            self.last_time = current_time

def main():
    rclpy.init()
    monitor = PerformanceMonitor()
    rclpy.spin(monitor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🔐 安全注意事项

1. **硬件安全**
   - 确保急停开关可用
   - 设置合理的关节限制
   - 定期检查机械连接

2. **软件安全**
   - 使用ros2_control的安全功能
   - 实现通信超时检测
   - 添加状态监控

3. **操作安全**
   - 首次运行时使用低速模式
   - 确保工作区域安全
   - 培训操作人员

---

## 📞 技术支持

如有问题，请：
1. 查看日志文件: `~/.ros/log/`
2. 检查GitHub Issues
3. 联系技术支持团队

祝您使用愉快！🎉
