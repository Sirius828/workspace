# 15Hz激光雷达扫描频率配置说明

## 配置概述

已经将oradar激光雷达的扫描频率从默认的10Hz提升到15Hz，以获得更高的数据密度和更好的建图效果。

## 启动方法

### 方法1：使用增强的mapping.launch.py（推荐）

```bash
# 使用默认15Hz频率启动
ros2 launch rb21_localization mapping.launch.py

# 或者显式指定频率
ros2 launch rb21_localization mapping.launch.py lidar_frequency:=15

# 如果需要降低频率，可以指定其他值
ros2 launch rb21_localization mapping.launch.py lidar_frequency:=10
```

### 方法2：使用独立的15Hz雷达launch文件

```bash
# 先启动15Hz雷达
ros2 launch rb21_localization oradar_15hz.launch.py

# 然后在另一个终端启动建图（不包含雷达启动）
ros2 launch rb21_localization mapping.launch.py
```

## 配置变更详情

### 1. 激光雷达配置
- **motor_speed**: 10Hz → 15Hz
- **预期效果**: 每秒15次360度扫描，提高数据密度

### 2. SLAM Toolbox优化配置
针对15Hz高频数据进行了以下优化：

```yaml
throttle_scans: 3              # 每3帧处理一次（15Hz→5Hz有效处理）
minimum_time_interval: 0.05   # 50ms最小时间间隔
map_update_interval: 1.5       # 1.5秒地图更新间隔
transform_timeout: 0.5         # 500ms变换超时
```

### 3. 数据流分析
- **原始频率**: 15Hz (66.7ms间隔)
- **处理频率**: 5Hz (每3帧处理一次)
- **有效数据利用率**: 33% (但数据质量更高)

## 性能优势

### 1. **更高的空间分辨率**
- 15Hz提供更密集的激光点云数据
- 在机器人移动时能捕获更多细节
- 减少因运动造成的数据丢失

### 2. **更好的建图质量**
- 更平滑的地图边界
- 更准确的障碍物轮廓
- 减少地图中的空白区域

### 3. **改善的定位精度**
- 更频繁的扫描匹配
- 提高在动态环境中的鲁棒性
- 更好的回环检测能力

## 潜在考虑

### 1. **计算负载**
- CPU使用率可能略有增加
- 建议在性能较好的设备上使用
- 通过throttle_scans参数已做优化

### 2. **存储空间**
- 更多的日志数据
- 地图文件可能略大
- 建议定期清理日志

### 3. **网络带宽**（如果使用网络传输）
- 数据量增加50%
- 建议在本地处理为主

## 监控和调试

### 查看激光雷达频率
```bash
# 查看扫描话题频率
ros2 topic hz /scan

# 查看过滤后的扫描频率  
ros2 topic hz /scan_filtered
```

### 监控SLAM性能
```bash
# 查看SLAM节点日志
ros2 node info /slam_toolbox

# 监控计算性能
top -p $(pgrep slam_toolbox)
```

## 故障排除

### 如果出现队列满警告
1. 确认15Hz频率设置正确
2. 检查throttle_scans设置
3. 考虑降低频率到12Hz或10Hz

### 如果建图效果不佳
1. 确保机器人移动速度适中
2. 检查TF树配置
3. 验证激光雷达数据质量

## 回退方案

如果15Hz造成问题，可以快速回退：

```bash
# 使用10Hz频率启动
ros2 launch rb21_localization mapping.launch.py lidar_frequency:=10
```

或者直接修改配置文件中的motor_speed参数。
