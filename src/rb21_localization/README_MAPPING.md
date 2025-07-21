# RB21 建图功能使用说明

## 文件说明

1. **mapping.launch.py** - 主要的建图启动文件
   - 启动EKF定位节点
   - 启动激光雷达过滤节点
   - 启动SLAM Toolbox建图节点

2. **config/laser_filter.yaml** - 激光雷达过滤配置
   - 配置为保留前方190度的激光数据
   - 适用于倒置安装的雷达

3. **config/slam_toolbox.yaml** - SLAM Toolbox配置
   - 同步SLAM模式配置
   - 针对室内建图优化的参数

## 使用方法

### 启动建图
```bash
# 确保ROS2环境已设置
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动建图
ros2 launch rb21_localization mapping.launch.py
```

### 保存地图
```bash
# 保存地图到指定路径
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: 
  data: 'my_map'"
```

### 查看建图状态
```bash
# 启动rviz2查看建图过程
rviz2
```

## 注意事项

1. 确保所有传感器节点已启动（IMU、轮式里程计、激光雷达）
2. 确保TF树正确配置
3. 建图过程中保持机器人缓慢移动，避免急转弯
4. 建图完成后及时保存地图

## 相关话题

- `/scan` - 原始激光雷达数据
- `/scan_filtered` - 过滤后的激光雷达数据  
- `/wheel/odom` - 轮式里程计
- `/imu/data_raw` - IMU数据
- `/map` - 构建的地图
- `/odom` - 过滤后的里程计
