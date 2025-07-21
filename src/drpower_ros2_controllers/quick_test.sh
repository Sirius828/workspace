#!/bin/bash

# DrPower ROS2 Controllers 快速测试脚本
# 用于验证系统是否正常工作

set -e

echo "🚀 DrPower ROS2 Controllers 快速测试"
echo "======================================"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未设置，请先运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS2环境: $ROS_DISTRO"

# 检查工作空间
if [ ! -f "install/setup.bash" ]; then
    echo "❌ 工作空间未编译，请先运行: colcon build"
    exit 1
fi

echo "✅ 工作空间已编译"

# 设置环境
source install/setup.bash

# 检查设备
DEVICE_PATH=${1:-/dev/ttyUSB0}
if [ ! -e "$DEVICE_PATH" ]; then
    echo "⚠️ 警告: CAN设备 $DEVICE_PATH 不存在"
    echo "请检查设备连接或指定正确的设备路径:"
    echo "  $0 /dev/ttyUSB1"
    echo ""
    echo "继续测试（仅验证软件功能）..."
else
    echo "✅ CAN设备: $DEVICE_PATH"
    
    # 检查设备权限
    if [ ! -r "$DEVICE_PATH" ] || [ ! -w "$DEVICE_PATH" ]; then
        echo "⚠️ 设备权限不足，尝试修复..."
        sudo chmod 666 "$DEVICE_PATH"
        echo "✅ 设备权限已修复"
    fi
fi

echo ""
echo "🔧 启动DrPower硬件接口..."
echo "================================"

# 启动硬件接口（后台运行）
ros2 launch drpower_hardware_interface drpower_arm.launch.py \
    device_path:=$DEVICE_PATH \
    baudrate:=115200 \
    use_sim_time:=false &

LAUNCH_PID=$!

# 等待系统启动
echo "⏳ 等待系统启动..."
sleep 5

# 检查控制器状态
echo ""
echo "📊 检查控制器状态..."
echo "=========================="

timeout 10s ros2 control list_controllers || {
    echo "❌ 控制器管理器未响应"
    kill $LAUNCH_PID 2>/dev/null || true
    exit 1
}

echo ""
echo "📡 检查硬件接口..."
echo "======================="

timeout 10s ros2 control list_hardware_interfaces || {
    echo "❌ 硬件接口未响应"
    kill $LAUNCH_PID 2>/dev/null || true
    exit 1
}

echo ""
echo "🎯 发送测试命令..."
echo "======================"

# 发送简单的关节命令
ros2 action send_goal --feedback /arm_joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
    "{
        trajectory: {
            joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            points: [{
                positions: [0.1, -0.1, 0.1, -0.1, 0.1, -0.1],
                time_from_start: {sec: 3}
            }]
        }
    }" &

ACTION_PID=$!

# 监控关节状态
echo ""
echo "📈 监控关节状态 (10秒)..."
echo "=========================="

timeout 10s bash -c '
while true; do
    echo -n "$(date +%H:%M:%S) - "
    ros2 topic echo --once /joint_states | grep -A1 "position:" | tail -1 || echo "无数据"
    sleep 1
done
' || echo "监控完成"

# 等待action完成
wait $ACTION_PID 2>/dev/null || true

echo ""
echo "🏁 测试完成"
echo "=============="

# 清理
kill $LAUNCH_PID 2>/dev/null || true
sleep 2

echo ""
echo "📋 测试总结:"
echo "- 如果看到控制器列表和硬件接口，说明软件功能正常"
echo "- 如果关节状态有数据更新，说明通信正常"
echo "- 如果action执行成功，说明控制功能正常"
echo ""
echo "🎉 如果一切正常，您的DrPower系统已准备就绪！"
echo ""
echo "下一步:"
echo "1. 运行完整测试: python3 examples/test_drpower_arm.py"
echo "2. 集成MoveIt2: ros2 launch your_moveit_config demo.launch.py"
echo "3. 自定义应用开发"

exit 0
