# YOLO检测器Launch文件使用指南

## 概述
这个launch文件为YOLO检测器提供了完全可配置的参数支持，允许您在运行时指定不同的模型、目标类别和其他配置。

## 基本用法

### 1. 使用默认参数启动
```bash
ros2 launch yolo_detector yolo_detector.launch.py
```
默认配置：
- 模型：ferrari.engine
- 目标类别：ferrari
- 置信度：0.85
- 设备：GPU (0)

### 2. 指定自定义模型和类别
```bash
# 使用YOLO11n模型检测人员
ros2 launch yolo_detector yolo_detector.launch.py \
    yolo_model:=yolo11n.pt \
    yolo_target_class:=person \
    yolo_confidence:=0.6

# 使用YOLO11s模型检测车辆
ros2 launch yolo_detector yolo_detector.launch.py \
    yolo_model:=yolo11s.pt \
    yolo_target_class:=car \
    yolo_confidence:=0.7

# 使用CPU推理
ros2 launch yolo_detector yolo_detector.launch.py \
    yolo_model:=yolo11n.pt \
    yolo_device:=cpu
```

## 可配置参数

| 参数名 | 默认值 | 描述 | 示例值 |
|--------|--------|------|--------|
| `yolo_model` | `ferrari.engine` | YOLO模型文件名 | `yolo11n.pt`, `custom_model.onnx` |
| `yolo_confidence` | `0.85` | 检测置信度阈值 | `0.6`, `0.7`, `0.9` |
| `yolo_target_class` | `ferrari` | 目标检测类别 | `person`, `car`, `truck`, `bicycle` |
| `yolo_device` | `0` | 推理设备 | `0` (GPU), `1` (第二块GPU), `cpu` |
| `input_image_topic` | `/camera/image_rect` | 输入图像话题 | `/camera/image_raw`, `/usb_cam/image_rect` |
| `output_position_topic` | `/target_position_pixel` | 输出位置话题 | `/detection/position`, `/target_center` |
| `output_result_topic` | `/yolo_detection_result` | 输出结果图像话题 | `/detection/result_image` |

## 支持的模型格式

### 1. TensorRT引擎文件 (.engine)
- **最佳性能**，GPU专用
- 示例：`ferrari.engine`, `person_detection.engine`

### 2. PyTorch模型 (.pt)
- **通用格式**，CPU/GPU兼容
- 示例：`yolo11n.pt`, `yolo11s.pt`, `yolo11m.pt`, `yolo11l.pt`, `yolo11x.pt`

### 3. ONNX模型 (.onnx)
- **跨平台**兼容
- 示例：`yolo11n.onnx`, `custom_model.onnx`

## 支持的目标类别

### COCO数据集标准类别（80类）
- **人和动物**: person, cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe
- **车辆**: bicycle, car, motorcycle, airplane, bus, train, truck, boat
- **日常物品**: bottle, wine glass, cup, fork, knife, spoon, bowl, banana, apple, orange
- **家具**: chair, couch, potted plant, bed, dining table, toilet, tv, laptop, mouse, remote
- **体育用品**: sports ball, kite, baseball bat, baseball glove, skateboard, surfboard, tennis racket
- **交通设施**: traffic light, fire hydrant, stop sign, parking meter, bench

### 自定义类别
- **Ferrari专用**: ferrari（需要专门训练的模型）
- **其他自定义**: 任何通过自定义数据集训练的特定目标

## 实际应用示例

### 1. 人员跟踪系统
```bash
ros2 launch yolo_detector yolo_detector.launch.py \
    yolo_model:=yolo11n.pt \
    yolo_target_class:=person \
    yolo_confidence:=0.6 \
    input_image_topic:=/camera/image_rect \
    output_position_topic:=/person_tracker/position
```

### 2. 车辆检测系统
```bash
ros2 launch yolo_detector yolo_detector.launch.py \
    yolo_model:=yolo11s.pt \
    yolo_target_class:=car \
    yolo_confidence:=0.7 \
    output_position_topic:=/vehicle_detector/position
```

### 3. 多目标检测（检测所有类别但只跟踪特定类别）
```bash
ros2 launch yolo_detector yolo_detector.launch.py \
    yolo_model:=yolo11m.pt \
    yolo_target_class:=truck \
    yolo_confidence:=0.5
```

### 4. 高精度检测（低噪声环境）
```bash
ros2 launch yolo_detector yolo_detector.launch.py \
    yolo_model:=yolo11x.pt \
    yolo_target_class:=person \
    yolo_confidence:=0.9
```

### 5. 快速检测（实时性要求高）
```bash
ros2 launch yolo_detector yolo_detector.launch.py \
    yolo_model:=yolo11n.pt \
    yolo_target_class:=person \
    yolo_confidence:=0.5
```

## 监控和调试

### 查看检测结果
```bash
# 监控目标位置输出
ros2 topic echo /target_position_pixel

# 查看检测结果图像（无数据数组）
ros2 topic echo /yolo_detection_result --no-arr

# 查看话题信息
ros2 topic info /target_position_pixel
ros2 topic info /yolo_detection_result

# 检查话题频率
ros2 topic hz /target_position_pixel
```

### 节点状态检查
```bash
# 查看运行的节点
ros2 node list

# 查看节点信息
ros2 node info /yolo_detector

# 查看参数
ros2 param list /yolo_detector
ros2 param get /yolo_detector model_path
ros2 param get /yolo_detector target_class
```

## 性能优化建议

### 1. 模型选择
- **高速度要求**: yolo11n.pt （最快，精度较低）
- **平衡性能**: yolo11s.pt （速度和精度平衡）
- **高精度要求**: yolo11m.pt, yolo11l.pt, yolo11x.pt （慢，但精度高）
- **最佳性能**: .engine格式（需要对应GPU的TensorRT转换）

### 2. 置信度阈值调整
- **高噪声环境**: 提高置信度到0.8-0.9
- **低光照条件**: 降低置信度到0.4-0.6
- **稳定环境**: 使用默认0.7-0.8

### 3. 设备选择
- **GPU推理**: 设置device:=0（或其他GPU编号）
- **CPU推理**: 设置device:=cpu（适合没有GPU的系统）

## 故障排除

### 常见问题
1. **模型文件找不到**: 确认模型文件在`src/yolo_detector/modules/`目录中
2. **虚拟环境问题**: 确认`~/ssd/ferrari/`虚拟环境存在且包含ultralytics
3. **GPU内存不足**: 尝试使用更小的模型或CPU推理
4. **检测精度低**: 调整置信度阈值或更换更大的模型

### 调试命令
```bash
# 检查虚拟环境
source ~/ssd/ferrari/bin/activate
python3 -c "import ultralytics; print('OK')"

# 检查模型文件
ls /home/sirius/ssd/ros2workspace/src/yolo_detector/modules/

# 检查GPU状态
nvidia-smi

# 查看详细日志
ros2 launch yolo_detector yolo_detector.launch.py --ros-args --log-level debug
```

## 与其他系统集成

这个YOLO检测器可以与以下系统无缝集成：
- **相机驱动**: 接收`/camera/image_rect`话题
- **云台控制**: 输出`/target_position_pixel`话题
- **机器人导航**: 提供目标位置信息
- **数据记录**: 记录检测结果用于分析

完整的集成示例请参考`ferrari_tracking_system.launch.py`。
