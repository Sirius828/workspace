# Vision Services Package

这是一个ROS2包，提供四种视觉服务用于检测图像中的正方形和矩形框。该包分为两个部分：
- `vision_services_msgs`：包含服务接口定义
- `vision_services`：包含Python实现

## 功能概述

### 订阅话题
- `/camera/color/image_raw` (sensor_msgs/msg/Image): 相机图像数据

### 提供的服务

1. **RESET** (`vision_services/reset`)
   - 识别图像中的铅笔正方形框
   - 计算并发布正方形框的中心坐标到 `/target_position`
   - 返回中心点坐标

2. **SQUARE_LOOP** (`vision_services/square_loop`)
   - 识别图像中的铅笔正方形框
   - 获取四个角点坐标，按顺序发布（1-2-3-4-1形成闭环）
   - 返回所有角点坐标

3. **A4_LOOP** (`vision_services/a4_loop`)
   - 识别图像中用黑色胶带拼接的矩形框
   - 计算胶带的中线矩形框
   - 发布中线矩形的角点坐标
   - 返回中线角点坐标

4. **A4_LOOP_ROT** (`vision_services/a4_loop_rot`)
   - 与A4_LOOP类似，但处理有旋转的矩形框
   - 使用最小外接矩形算法
   - 返回中线角点坐标和旋转角度

### 发布话题
- `/target_position` (geometry_msgs/msg/Point): 目标位置（RESET服务）
- `/corner_points` (geometry_msgs/msg/Point32): 角点坐标（SQUARE_LOOP服务）
- `/midline_points` (geometry_msgs/msg/Point32): 中线点坐标（A4_LOOP和A4_LOOP_ROT服务）

## 安装和使用

### 1. 构建包
```bash
cd /home/sirius/ssd/ros2workspace
# 首先构建消息包
colcon build --packages-select vision_services_msgs
# 然后构建主包
colcon build --packages-select vision_services
source install/setup.bash
```

### 2. 启动节点
```bash
# 使用launch文件启动（基本模式）
ros2 launch vision_services vision_services.launch.py

# 启用预览界面
ros2 launch vision_services vision_services.launch.py preview:=true

# 启用调试模式
ros2 launch vision_services vision_services.launch.py debug:=true preview:=true

# 或直接启动节点
ros2 run vision_services vision_services_node
```

### 3. 预览界面
包提供了丰富的可视化预览界面，方便调试和监控：

```bash
# 运行预览演示
ros2 run vision_services preview_demo
```

预览界面包括：
- **原始图像窗口**：显示相机输入
- **预处理图像窗口**：显示边缘检测结果
- **检测结果窗口**：显示检测到的图形，包含：
  - 正方形/矩形轮廓（绿色/蓝色）
  - 中心点和角点标记
  - 实时信息面板（FPS、检测时间等）

### 4. 调用服务

#### 命令行调用
```bash
# RESET服务
ros2 service call /vision_services/reset vision_services_msgs/srv/Reset

# SQUARE_LOOP服务
ros2 service call /vision_services/square_loop vision_services_msgs/srv/SquareLoop

# A4_LOOP服务
ros2 service call /vision_services/a4_loop vision_services_msgs/srv/A4Loop

# A4_LOOP_ROT服务
ros2 service call /vision_services/a4_loop_rot vision_services_msgs/srv/A4LoopRot
```

#### 使用测试脚本
```bash
# 测试所有服务
ros2 run vision_services test_client

# 测试特定服务
ros2 run vision_services test_client reset
ros2 run vision_services test_client square_loop
ros2 run vision_services test_client a4_loop
ros2 run vision_services test_client a4_loop_rot
```

## 配置文件

配置文件位于 `config/vision_config.yaml`，包含以下可调参数：

### 图像预处理参数
- `gaussian_blur`: 高斯模糊参数
- `morphology`: 形态学操作参数
- `canny`: 边缘检测参数

### 检测参数
- `square_detection`: 正方形检测相关参数
- `a4_detection`: A4矩形检测相关参数
- `rotated_rectangle`: 旋转矩形检测参数

### 发布参数
- `publishing`: 话题名称和坐标系配置

### 调试参数
- `debug`: 调试模式和日志级别

## 依赖

### ROS2包依赖
- `rclpy`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`
- `vision_services_msgs`

### Python依赖
- `opencv-python`
- `numpy`
- `pyyaml`

## 包结构

```
vision_services_msgs/
├── CMakeLists.txt
├── package.xml
└── srv/
    ├── Reset.srv
    ├── SquareLoop.srv
    ├── A4Loop.srv
    └── A4LoopRot.srv

vision_services/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── vision_services
├── config/
│   └── vision_config.yaml
├── launch/
│   └── vision_services.launch.py
├── test/
│   └── test_vision_services.py
├── vision_services/
│   ├── __init__.py
│   ├── vision_services_node.py
│   ├── image_processor.py
│   └── test_client.py
└── README.md
```

## 算法说明

### 正方形检测
1. 图像预处理（灰度化、高斯模糊、边缘检测）
2. 轮廓检测
3. 多边形逼近
4. 正方形验证（边长比例、角度检查）

### 矩形检测
1. HSV颜色空间转换
2. 黑色胶带掩码生成
3. 形态学操作
4. 轮廓检测和验证
5. 中线计算

### 旋转矩形检测
1. 基于矩形检测的流程
2. 使用最小外接矩形
3. 角度计算和返回

## 调试

启用调试模式：
```bash
ros2 launch vision_services vision_services.launch.py debug:=true
```

调试模式会显示处理过程中的图像窗口，帮助调试算法参数。

## 故障排除

1. **没有检测到图形**
   - 检查相机是否正常工作
   - 调整配置文件中的检测参数
   - 启用调试模式查看处理结果

2. **检测精度不够**
   - 调整图像预处理参数
   - 修改轮廓检测的面积阈值
   - 调整验证参数

3. **服务调用失败**
   - 确保节点正在运行
   - 检查服务名称是否正确
   - 查看日志输出

4. **依赖问题**
   - 确保安装了所有Python依赖：`pip install opencv-python numpy pyyaml`
   - 确保构建了消息包：`colcon build --packages-select vision_services_msgs`
