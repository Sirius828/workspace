# Gimbal Pixel Controller

基于像素误差的云台跟踪控制器，用于视觉伺服控制。

## 功能特性

- **PID控制**：独立的YAW和PITCH轴PID控制器
- **位置控制适配**：针对位置控制云台的累积角度控制
- **安全限制**：角度限制、变化率限制、置信度阈值
- **超时保护**：目标丢失时的安全停止机制
- **参数调节**：可调节的控制参数

## 控制原理

```
目标像素位置 -> 像素误差 -> PID控制器 -> 角度增量 -> 累积角度 -> 云台位置指令
     ↑              ↑           ↑           ↑           ↑            ↑
  检测结果        中心点差值   比例积分微分  限制输出    角度累积      /cmd_gimbal
```

## 话题接口

### 订阅的话题
- `/target_position_pixel` (geometry_msgs/Point): 目标像素位置
  - `x`: 目标x坐标 (像素)
  - `y`: 目标y坐标 (像素)
  - `z`: 检测置信度 (0-1)

### 发布的话题
- `/cmd_gimbal` (geometry_msgs/Twist): 云台控制指令
  - `angular.x`: 云台yaw角度 (弧度)
  - `angular.y`: 云台pitch角度 (弧度)

## 参数配置

### 图像参数
- `image_width`: 图像宽度 (默认: 640)
- `image_height`: 图像高度 (默认: 480)
- `target_pixel_x`: 目标像素x坐标，通常是图像中心 (默认: 320)
- `target_pixel_y`: 目标像素y坐标，通常是图像中心 (默认: 240)

### PID参数
- `yaw_kp`, `yaw_ki`, `yaw_kd`: YAW轴PID参数
- `pitch_kp`, `pitch_ki`, `pitch_kd`: PITCH轴PID参数

### 控制限制
- `max_angle_change`: 单次最大角度变化 (弧度)
- `min_confidence`: 最小置信度阈值
- `control_timeout`: 控制超时时间 (秒)
- `yaw_min/max`: YAW轴角度限制 (弧度)
- `pitch_min/max`: PITCH轴角度限制 (弧度)

## 使用方法

### 编译
```bash
cd /home/sirius/ssd/ros2workspace
colcon build --packages-select gimbal_pixel_controller
source install/setup.bash
```

### 运行控制器
```bash
# 使用默认参数
ros2 launch gimbal_pixel_controller gimbal_pixel_controller.launch.py

# 或使用自定义参数文件
ros2 launch gimbal_pixel_controller gimbal_pixel_controller.launch.py params_file:=/path/to/your/params.yaml
```

### 测试模式
```bash
# 运行测试节点（生成模拟目标）
ros2 run gimbal_pixel_controller test_node
```

## 参数调节指南

### 1. 基础调节
1. 首先确保图像尺寸参数正确
2. 设置合适的云台角度限制
3. 从较小的P增益开始 (0.001-0.005)

### 2. PID调节步骤
1. **比例增益 (Kp)**：
   - 从小值开始，逐渐增加
   - 过小：响应慢
   - 过大：震荡
   
2. **积分增益 (Ki)**：
   - 消除稳态误差
   - 通常比Kp小一个数量级
   
3. **微分增益 (Kd)**：
   - 减少超调
   - 提高稳定性

### 3. 常见问题解决
- **震荡**：减小Kp，增加Kd
- **响应慢**：增加Kp
- **稳态误差**：增加Ki
- **超调**：增加Kd，减小Kp

## 系统集成

### 与YOLO检测器集成
```bash
# 终端1：启动YOLO检测器
ros2 run yolo_detector detector_node

# 终端2：启动云台控制器
ros2 launch gimbal_pixel_controller gimbal_pixel_controller.launch.py

# 终端3：启动底盘控制器 (如果需要)
ros2 launch chassis_hardware chassis_hardware.launch.py
```

### 监控和调试
```bash
# 监控目标检测
ros2 topic echo /target_position_pixel

# 监控云台指令
ros2 topic echo /cmd_gimbal

# 调整参数 (运行时)
ros2 param set /gimbal_pixel_controller yaw_kp 0.005
```

## 注意事项

1. **相机标定**：需要根据实际相机视场角调整像素到角度的映射
2. **延迟补偿**：如果系统延迟较大，可能需要增加预测控制
3. **平滑滤波**：如果检测结果抖动严重，可以添加滤波器
4. **动态调节**：根据目标距离动态调整控制增益

## 扩展功能

可以根据需要添加：
- 自适应PID参数
- 卡尔曼滤波预测
- 多目标优先级选择
- 运动补偿
- 图像稳定化
