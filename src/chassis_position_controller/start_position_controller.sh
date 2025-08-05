#!/bin/bash

# 机器人位置控制器启动脚本
# 简化版本，只启动核心控制器

cd /home/sirius/ssd/ros2workspace

# 检查是否已经source环境
if [ -z "$ROS_DISTRO" ]; then
    echo "🔧 Sourcing ROS2 environment..."
    source /opt/ros/humble/setup.bash
    source install/setup.bash
fi

echo "🚀 Starting Position Controller..."
echo "使用方法："
echo "  ros2 launch chassis_position_controller position_controller.launch.py"
echo "  或者直接运行: ros2 run chassis_position_controller position_controller"
echo ""

# 解析参数
LAUNCH_MODE="launch"
SIMULATION=false
HARDWARE=false
RVIZ=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --direct|-d)
            LAUNCH_MODE="direct"
            shift
            ;;
        --sim|-s)
            SIMULATION=true
            shift
            ;;
        --hw|-h)
            HARDWARE=true
            shift
            ;;
        --rviz|-r)
            RVIZ=true
            shift
            ;;
        *)
            echo "未知参数: $1"
            exit 1
            ;;
    esac
done

# 启动仿真（如果需要）
if [ "$SIMULATION" = true ]; then
    echo "🎮 Starting chassis simulation..."
    gnome-terminal -- bash -c "cd /home/sirius/ssd/ros2workspace && source install/setup.bash && ros2 launch chassis_simulation chassis_simulation.launch.py; exec bash"
    sleep 3
fi

# 启动硬件接口（如果需要）
if [ "$HARDWARE" = true ]; then
    echo "🔧 Starting chassis hardware interface..."
    gnome-terminal -- bash -c "cd /home/sirius/ssd/ros2workspace && source install/setup.bash && ros2 launch chassis_hardware chassis_hardware.launch.py; exec bash"
    sleep 3
fi

# 启动RViz（如果需要）
if [ "$RVIZ" = true ]; then
    echo "👁️ Starting RViz2..."
    gnome-terminal -- bash -c "cd /home/sirius/ssd/ros2workspace && source install/setup.bash && rviz2; exec bash"
    sleep 2
fi

# 启动位置控制器
if [ "$LAUNCH_MODE" = "launch" ]; then
    echo "🎯 Starting position controller via launch file..."
    ros2 launch chassis_position_controller position_controller.launch.py
else
    echo "🎯 Starting position controller directly..."
    ros2 run chassis_position_controller position_controller
fi
