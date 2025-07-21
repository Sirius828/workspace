# 底盘测试指南

## 🚀 快速测试步骤

### 1. 启动完整系统
首先在一个终端启动完整的机器人系统：
```bash
cd /home/sirius/ssd/ros2workspace
source install/setup.bash
ros2 launch ti_diffbot_hardware gimbal_complete_system.launch.py
```

### 2. 运行底盘测试
在另一个终端运行底盘功能测试：
```bash
cd /home/sirius/ssd/ros2workspace
source install/setup.bash
ros2 run ti_diffbot_hardware test_chassis.py
```

或者使用launch文件（延时启动）：
```bash
ros2 launch ti_diffbot_hardware test_chassis.launch.py
```

## 📊 测试内容

测试脚本会自动执行以下测试序列：

1. **初始状态检查** - 验证传感器数据是否正常
2. **前进运动** - 3秒钟0.1m/s前进
3. **后退运动** - 3秒钟0.1m/s后退  
4. **左转运动** - 3秒钟0.5rad/s左转
5. **右转运动** - 3秒钟0.5rad/s右转
6. **弧线运动** - 4秒钟组合运动

每个测试后会显示详细的状态报告，包括：
- 🗺️ 里程计位置和速度
- ⚙️ 左右轮关节位置
- 🔄 左右轮关节速度

## 🔍 监控话题

在测试过程中，你可以在额外的终端监控话题：

```bash
# 监控里程计
ros2 topic echo /diff_drive_controller/odom

# 监控关节状态
ros2 topic echo /joint_states

# 监控速度命令
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped

# 检查控制器状态
ros2 control list_controllers
```

## ⚠️ 注意事项

- 确保硬件连接正常且串口通信稳定
- 测试期间请确保机器人有足够的移动空间
- 如果发现异常，可以按Ctrl+C中断测试
- 测试完成后机器人会自动停止

## 🛠️ 故障排除

1. **未收到传感器数据**
   - 检查硬件连接
   - 确认控制器状态：`ros2 control list_controllers`
   - 查看节点日志排查问题

2. **机器人不响应命令**
   - 确认diff_drive_controller状态为active
   - 检查话题连接：`ros2 topic info /diff_drive_controller/cmd_vel_unstamped`

3. **数据异常**
   - 查看硬件接口日志：`ros2 node list` 然后 `ros2 node info <node_name>`
   - 检查串口通信状态
