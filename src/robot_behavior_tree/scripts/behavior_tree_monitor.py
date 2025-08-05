#!/usr/bin/env python3

"""
机器人行为树监控和控制脚本

这个脚本提供了一个简单的命令行界面来监控和控制机器人的行为树执行。
功能包括：
1. 实时状态监控
2. 任务启动/停止
3. 行为树文件切换
4. 系统诊断
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import sys
import threading
import time
from typing import Dict, List, Optional

# ROS2 message types
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

# Service types
from std_srvs.srv import SetBool

class BehaviorTreeMonitor(Node):
    def __init__(self):
        super().__init__('behavior_tree_monitor')
        
        # Status variables
        self.bt_status = "UNKNOWN"
        self.robot_position = Point()
        self.target_detected = False
        self.target_position = Point()
        self.gimbal_tracking = False
        
        # QoS profiles
        self.reliable_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.best_effort_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.bt_status_sub = self.create_subscription(
            String, '/behavior_tree/status', self.bt_status_callback, self.reliable_qos)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/chassis/odom', self.odom_callback, self.best_effort_qos)
        
        self.target_sub = self.create_subscription(
            Point, '/target_position_pixel', self.target_callback, self.best_effort_qos)
        
        # Publishers
        self.bt_command_pub = self.create_publisher(
            String, '/behavior_tree/command', self.reliable_qos)
        
        # Service clients
        self.bt_control_client = self.create_client(SetBool, '/behavior_tree/control')
        
        # Status tracking
        self.last_status_time = time.time()
        self.status_history: List[str] = []
        
        self.get_logger().info("BehaviorTree Monitor initialized")
    
    def bt_status_callback(self, msg):
        """行为树状态回调"""
        self.bt_status = msg.data
        self.last_status_time = time.time()
        self.status_history.append(f"{time.strftime('%H:%M:%S')} - {msg.data}")
        
        # Keep only last 50 status messages
        if len(self.status_history) > 50:
            self.status_history.pop(0)
    
    def odom_callback(self, msg):
        """里程计回调"""
        self.robot_position.x = msg.pose.pose.position.x
        self.robot_position.y = msg.pose.pose.position.y
        self.robot_position.z = msg.pose.pose.position.z
    
    def target_callback(self, msg):
        """目标检测回调"""
        self.target_detected = True
        self.target_position = msg
        # Reset detection flag after timeout
        threading.Timer(2.0, lambda: setattr(self, 'target_detected', False)).start()
    
    def send_command(self, command: str) -> bool:
        """发送命令到行为树执行器"""
        try:
            msg = String()
            msg.data = command
            self.bt_command_pub.publish(msg)
            self.get_logger().info(f"Sent command: {command}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")
            return False
    
    def control_bt_execution(self, enable: bool) -> bool:
        """控制行为树执行"""
        if not self.bt_control_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("BehaviorTree control service not available")
            return False
        
        try:
            request = SetBool.Request()
            request.data = enable
            
            future = self.bt_control_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result():
                result = future.result()
                self.get_logger().info(f"Control result: {result.message}")
                return result.success
            else:
                self.get_logger().error("Service call failed")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Service call exception: {e}")
            return False
    
    def get_system_status(self) -> Dict[str, str]:
        """获取系统状态信息"""
        current_time = time.time()
        time_since_last_status = current_time - self.last_status_time
        
        status = {
            "BehaviorTree Status": self.bt_status,
            "Last Update": f"{time_since_last_status:.1f}s ago",
            "Robot Position": f"({self.robot_position.x:.2f}, {self.robot_position.y:.2f})",
            "Target Detected": "YES" if self.target_detected else "NO",
            "Target Position": f"({self.target_position.x:.0f}, {self.target_position.y:.0f})" if self.target_detected else "N/A",
            "Target Confidence": f"{self.target_position.z:.2f}" if self.target_detected else "N/A"
        }
        
        return status
    
    def print_status(self):
        """打印当前状态"""
        status = self.get_system_status()
        
        print("\n" + "="*60)
        print("🤖 ROBOT BEHAVIOR TREE MONITOR")
        print("="*60)
        
        for key, value in status.items():
            print(f"{key:<20}: {value}")
        
        print("\n📊 Recent Status History:")
        for history_item in self.status_history[-5:]:
            print(f"   {history_item}")
        
        print("="*60)
    
    def run_interactive_monitor(self):
        """运行交互式监控"""
        print("\n🚀 Starting Interactive BehaviorTree Monitor")
        print("Commands:")
        print("  's' - Start BehaviorTree")
        print("  'x' - Stop BehaviorTree") 
        print("  'r' - Reset BehaviorTree")
        print("  'p' - Show current status")
        print("  'h' - Show status history")
        print("  'l' - Load behavior tree file")
        print("  'q' - Quit")
        print("-" * 50)
        
        while rclpy.ok():
            try:
                command = input("\nEnter command: ").strip().lower()
                
                if command == 'q':
                    break
                elif command == 's':
                    print("🟢 Starting BehaviorTree...")
                    success = self.control_bt_execution(True)
                    print("✅ Started" if success else "❌ Failed to start")
                    
                elif command == 'x':
                    print("🔴 Stopping BehaviorTree...")
                    success = self.control_bt_execution(False)
                    print("✅ Stopped" if success else "❌ Failed to stop")
                    
                elif command == 'r':
                    print("🔄 Resetting BehaviorTree...")
                    success = self.send_command("reset")
                    print("✅ Reset" if success else "❌ Failed to reset")
                    
                elif command == 'p':
                    self.print_status()
                    
                elif command == 'h':
                    print("\n📈 Status History:")
                    for item in self.status_history:
                        print(f"   {item}")
                    
                elif command == 'l':
                    file_path = input("Enter behavior tree file path: ").strip()
                    if file_path:
                        print(f"📁 Loading file: {file_path}")
                        success = self.send_command(f"load:{file_path}")
                        print("✅ Loaded" if success else "❌ Failed to load")
                    
                else:
                    print("❓ Unknown command. Type 'q' to quit.")
                
                # Brief status update after each command
                if command in ['s', 'x', 'r']:
                    time.sleep(1)  # Give time for status to update
                    print(f"Current Status: {self.bt_status}")
                
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        print("\n👋 Monitor exiting...")

def main():
    rclpy.init()
    
    monitor = BehaviorTreeMonitor()
    
    # Start monitor in a separate thread
    def spin_node():
        rclpy.spin(monitor)
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    # Wait a moment for connections
    time.sleep(2)
    
    try:
        if len(sys.argv) > 1 and sys.argv[1] == "--status-only":
            # Just print status and exit
            monitor.print_status()
        else:
            # Run interactive monitor
            monitor.run_interactive_monitor()
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
