# Gimbal Description Package

这个包包含了云台系统的URDF模型文件，用于ROS2机器人描述。

## 包结构

```
gimbal_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── gimbal.rviz        # RViz配置文件
├── launch/
│   ├── display.launch.py  # 启动文件（带GUI）
│   └── robot_state_publisher.launch.py  # 启动文件（不带GUI）
├── meshes/                # 3D模型文件（可选）
└── urdf/
    └── gimbal.urdf.xacro  # 云台URDF模型文件
```

## 云台TF树结构

```
screen_frame (静态，从标定得到)
└── camera_link (静态)
    └── gimbal_base_link (静态，相机→云台底座外参)
        └── gimbal_pan_link (动态，θ_pan - pan_joint)
            └── gimbal_tilt_link (动态，θ_tilt - tilt_joint)
                └── laser_link (静态，光心小偏移)
```

## 关节和链接说明

### 静态变换
- **screen_frame → camera_link**: 从屏幕坐标系到相机坐标系的标定变换
- **camera_link → gimbal_base_link**: 相机到云台底座的外参变换
- **gimbal_tilt_link → laser_link**: 激光器的光心小偏移

### 动态关节
- **pan_joint**: 水平旋转关节，绕Z轴旋转，范围：-π 到 π，订阅joint_states话题
- **tilt_joint**: 俯仰旋转关节，绕Y轴旋转，范围：-π/2 到 π/2，订阅joint_states话题

## 偏移参数配置

在URDF文件开头定义了所有需要配置的偏移参数，请根据实际标定结果修改：

```xml
<!-- screen_frame到camera_link的静态变换 (从标定得到) -->
<xacro:property name="screen_to_camera_x" value="0.0"/>
<xacro:property name="screen_to_camera_y" value="0.0"/>
<xacro:property name="screen_to_camera_z" value="0.0"/>
<xacro:property name="screen_to_camera_roll" value="0.0"/>
<xacro:property name="screen_to_camera_pitch" value="0.0"/>
<xacro:property name="screen_to_camera_yaw" value="0.0"/>

<!-- camera_link到gimbal_base_link的静态变换 (相机→云台底座外参) -->
<xacro:property name="camera_to_gimbal_base_x" value="0.0"/>
<xacro:property name="camera_to_gimbal_base_y" value="0.0"/>
<xacro:property name="camera_to_gimbal_base_z" value="0.0"/>
<xacro:property name="camera_to_gimbal_base_roll" value="0.0"/>
<xacro:property name="camera_to_gimbal_base_pitch" value="0.0"/>
<xacro:property name="camera_to_gimbal_base_yaw" value="0.0"/>

<!-- laser_link的光心小偏移 -->
<xacro:property name="tilt_to_laser_x" value="0.0"/>
<xacro:property name="tilt_to_laser_y" value="0.0"/>
<xacro:property name="tilt_to_laser_z" value="0.06"/>
<xacro:property name="tilt_to_laser_roll" value="0.0"/>
<xacro:property name="tilt_to_laser_pitch" value="0.0"/>
<xacro:property name="tilt_to_laser_yaw" value="0.0"/>
```

## 使用方法

### 1. 编译包

```bash
cd /home/sirius/Documents/ros2WorkspAce
colcon build --packages-select gimbal_description
source install/setup.bash
```

### 2. 启动云台模型显示（带GUI）

```bash
ros2 launch gimbal_description display.launch.py
```

这将启动：
- robot_state_publisher：发布机器人状态
- joint_state_publisher_gui：提供关节控制界面
- rviz2：3D可视化

### 3. 启动云台模型发布（不带GUI）

```bash
ros2 launch gimbal_description robot_state_publisher.launch.py
```

这将启动：
- robot_state_publisher：发布机器人状态
- joint_state_publisher：订阅joint_states话题

### 4. 查看TF树

```bash
ros2 run tf2_tools view_frames
```

### 5. 控制云台关节

云台的两个旋转关节订阅`joint_states`话题：

```bash
# 发布关节状态 (θ_pan=0.5弧度, θ_tilt=0.3弧度)
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {stamp: now, frame_id: ''}, 
  name: ['pan_joint', 'tilt_joint'], 
  position: [0.5, 0.3], 
  velocity: [], 
  effort: []
}"
```

### 6. 实时监控关节状态

```bash
ros2 topic echo /joint_states
```

## 自定义配置

### 修改偏移参数

编辑 `urdf/gimbal.urdf.xacro` 文件开头的偏移参数：

1. **标定参数**: 根据相机标定结果修改`screen_to_camera_*`参数
2. **外参参数**: 根据相机到云台底座的外参修改`camera_to_gimbal_base_*`参数
3. **光心偏移**: 根据激光器安装位置修改`tilt_to_laser_*`参数

### 修改关节限制

```xml
<!-- 修改水平旋转范围 -->
<xacro:property name="pan_joint_lower_limit" value="-3.14159"/>
<xacro:property name="pan_joint_upper_limit" value="3.14159"/>

<!-- 修改俯仰旋转范围 -->
<xacro:property name="tilt_joint_lower_limit" value="-1.57079"/>
<xacro:property name="tilt_joint_upper_limit" value="1.57079"/>
```

### 添加3D模型

1. 将STL或DAE文件放入 `meshes/` 目录
2. 在URDF文件中引用：

```xml
<visual>
  <geometry>
    <mesh filename="package://gimbal_description/meshes/your_model.stl"/>
  </geometry>
</visual>
```

## 依赖项

- urdf
- xacro
- robot_state_publisher
- joint_state_publisher
- joint_state_publisher_gui
- rviz2

## 注意事项

- 所有偏移参数都在URDF文件开头定义，便于修改
- 两个旋转关节（pan_joint和tilt_joint）都订阅joint_states话题
- 固定的TF变换（screen_frame→camera_link→gimbal_base_link→laser_link）
- 根据实际硬件调整所有偏移参数和关节限制
- 如果需要在Gazebo中使用，可能需要添加额外的插件和属性
