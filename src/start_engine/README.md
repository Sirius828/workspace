# Start Engine GUI

一个简单的GUI包，提供一键启动按钮功能来调用 `/start` 服务。

## 功能特点

- 简洁的图形用户界面
- 一键启动按钮
- 服务状态反馈
- 进度指示
- 错误处理和用户提示

## 使用方法

### 构建包
```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select start_engine
source install/setup.bash
```

### 运行GUI
```bash
ros2 run start_engine start_gui
```

### 功能说明

1. **启动按钮**: 点击"启动系统"按钮来调用 `/start` 服务
2. **状态显示**: 界面顶部显示当前状态（准备就绪/正在启动/启动成功/启动失败等）
3. **进度指示**: 服务调用过程中显示进度条
4. **消息提示**: 成功、失败或警告时弹出相应的消息框

### 服务要求

GUI尝试调用名为 `/start` 的服务，该服务应该是 `std_srvs/Empty` 类型。

## 后续扩展

当前版本只包含基本的启动按钮功能。后续可以轻松添加：
- 更多控制按钮
- 系统状态监控
- 参数配置界面
- 日志显示
- 多服务支持

## 依赖项

- rclpy
- std_srvs
- tkinter (Python标准库)

## 注意事项

- 如果 `/start` 服务不可用，GUI会显示相应的警告消息
- GUI在后台线程中运行ROS2节点，不会阻塞界面响应
- 支持优雅的关闭处理
