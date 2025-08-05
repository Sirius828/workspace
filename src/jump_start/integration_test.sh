#!/bin/bash
# Jump Start + Start Engine 集成测试脚本

echo "🤖 Jump Start + Start Engine Integration Test"
echo "=============================================="

cd /home/sirius/ssd/ros2workspace
source install/setup.bash

echo "📋 Available test options:"
echo "1. Test /start service (FSM only)"
echo "2. Launch integrated system (FSM + GUI)"
echo "3. Test integration automatically"
echo "4. Check service availability"

read -p "Enter your choice (1-4): " choice

case $choice in
    1)
        echo "🧪 Testing /start service..."
        echo "Starting FSM controller in background..."
        ros2 run jump_start fsm_controller &
        FSM_PID=$!
        
        sleep 3
        echo "Calling /start service..."
        ros2 service call /start std_srvs/srv/Empty
        
        echo "Stopping FSM controller..."
        kill $FSM_PID
        ;;
    2)
        echo "🚀 Launching integrated system..."
        echo "This will start both FSM controller and start_engine GUI"
        ros2 launch jump_start integrated_system.launch.py
        ;;
    3)
        echo "🧪 Running automatic integration test..."
        echo "Starting FSM controller in background..."
        ros2 run jump_start fsm_controller &
        FSM_PID=$!
        
        sleep 3
        echo "Running integration test..."
        python3 src/jump_start/test_integration.py
        
        echo "Stopping FSM controller..."
        kill $FSM_PID
        ;;
    4)
        echo "🔍 Checking service availability..."
        echo "Listing available services..."
        ros2 service list | grep -E "(start|fsm)"
        echo ""
        echo "Service info for /start:"
        timeout 5 ros2 service info /start 2>/dev/null || echo "Service not available"
        ;;
    *)
        echo "❌ Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "✅ Test completed!"
echo ""
echo "📖 Usage summary:"
echo "  - GUI启动: ros2 launch jump_start integrated_system.launch.py"
echo "  - 服务调用: ros2 service call /start std_srvs/srv/Empty"
echo "  - 状态监控: ros2 topic echo /fsm_state"
