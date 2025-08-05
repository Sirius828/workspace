#!/bin/bash
# 位置跟踪演示脚本

echo "🤖 Position-Based Tracking Demo"
echo "==============================="

cd /home/sirius/ssd/ros2workspace
source install/setup.bash

echo "📋 Available demos:"
echo "1. Start FSM Controller"
echo "2. Run Position Tracking Test"
echo "3. Manual Position Test"
echo "4. Monitor System Status"

read -p "Enter your choice (1-4): " choice

case $choice in
    1)
        echo "🚀 Starting FSM Controller with position tracking..."
        ros2 run jump_start fsm_controller
        ;;
    2)
        echo "🧪 Running automated position tracking test..."
        echo "⚠️  Make sure FSM Controller is running in another terminal!"
        read -p "Press Enter to continue when FSM Controller is ready..."
        python3 src/jump_start/position_tracking_test.py
        ;;
    3)
        echo "🎮 Manual Position Test"
        echo "Commands to try in separate terminals:"
        echo ""
        echo "# Terminal 1: Start FSM Controller"
        echo "ros2 run jump_start fsm_controller"
        echo ""
        echo "# Terminal 2: Set robot position (invalid zone x=1.5)"
        echo "ros2 topic pub --once /chassis/odom nav_msgs/msg/Odometry \"{"
        echo "  header: {frame_id: 'odom'},"
        echo "  pose: {pose: {position: {x: 1.5, y: 2.0, z: 0.0}}}"
        echo "}\""
        echo ""
        echo "# Terminal 3: Start tracking"
        echo "ros2 topic pub --once /mission_command std_msgs/msg/String \"data: 'track'\""
        echo ""
        echo "# Terminal 4: Publish target in center (should NOT trigger victory)"
        echo "ros2 topic pub -r 10 /target_position_pixel geometry_msgs/msg/Point \"x: 320.0, y: 240.0, z: 0.9\""
        echo ""
        echo "# Terminal 5: Move to valid zone (x=2.5)"
        echo "ros2 topic pub --once /chassis/odom nav_msgs/msg/Odometry \"{"
        echo "  header: {frame_id: 'odom'},"
        echo "  pose: {pose: {position: {x: 2.5, y: 2.0, z: 0.0}}}"
        echo "}\""
        echo ""
        echo "# Now target tracking should work and trigger victory after 2 seconds"
        ;;
    4)
        echo "📊 Monitoring system status..."
        echo "FSM State:"
        timeout 3 ros2 topic echo /fsm_state --once 2>/dev/null || echo "No FSM state available"
        echo ""
        echo "FSM Status (detailed):"
        timeout 3 ros2 topic echo /fsm_status --once 2>/dev/null || echo "No FSM status available"
        echo ""
        echo "Victory Status:"
        timeout 3 ros2 topic echo /victory --once 2>/dev/null || echo "No victory status available"
        ;;
    *)
        echo "❌ Invalid choice"
        exit 1
        ;;
esac
