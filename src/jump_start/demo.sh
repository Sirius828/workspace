#!/bin/bash
# Jump Start 使用示例

echo "🤖 Jump Start Package Demo"
echo "=========================="

# 检查ROS2环境
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found. Please source ROS2 setup."
    exit 1
fi

cd /home/sirius/ssd/ros2workspace
source install/setup.bash

echo "📋 Available commands:"
echo "1. Start FSM Controller"
echo "2. Start Mission Executor"
echo "3. Run Simple Demo"
echo "4. Test Navigation Command"
echo "5. Test Tracking Command"
echo "6. Monitor System Status"

read -p "Enter your choice (1-6): " choice

case $choice in
    1)
        echo "🚀 Starting FSM Controller..."
        ros2 run jump_start fsm_controller
        ;;
    2)
        echo "🎮 Starting Mission Executor..."
        ros2 run jump_start mission_executor
        ;;
    3)
        echo "🎯 Running Simple Demo..."
        python3 src/jump_start/jump_start/simple_demo.py
        ;;
    4)
        echo "📍 Testing Navigation Command..."
        echo "Sending navigation command: navigate 1.0 1.0 0.3"
        ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'navigate 1.0 1.0 0.3'"
        ;;
    5)
        echo "🎯 Testing Tracking Command..."
        echo "Sending tracking command: track"
        ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'track'"
        ;;
    6)
        echo "📊 Monitoring System Status..."
        echo "FSM State:"
        timeout 5 ros2 topic echo /fsm_state --once
        echo "FSM Status:"
        timeout 5 ros2 topic echo /fsm_status --once
        ;;
    *)
        echo "❌ Invalid choice"
        exit 1
        ;;
esac
