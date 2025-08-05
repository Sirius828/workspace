#!/usr/bin/env python3
"""
任务执行器节点 - 提供高级任务接口
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time
from typing import List, Tuple

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point


class MissionExecutor(Node):
    """任务执行器 - 封装常用的机器人任务"""
    
    def __init__(self):
        super().__init__('mission_executor')
        
        # QoS配置
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 发布者
        self.mission_command_pub = self.create_publisher(
            String,
            '/mission_command',
            qos_reliable
        )
        
        # 订阅者
        self.fsm_state_sub = self.create_subscription(
            String,
            '/fsm_state',
            self.fsm_state_callback,
            qos_reliable
        )
        
        self.victory_sub = self.create_subscription(
            Bool,
            '/victory',
            self.victory_callback,
            qos_reliable
        )
        
        # 状态追踪
        self.current_fsm_state = "IDLE"
        self.mission_completed = False
        self.victory_achieved = False
        
        self.get_logger().info('Mission Executor started')
    
    def fsm_state_callback(self, msg: String):
        """FSM状态回调"""
        self.current_fsm_state = msg.data
    
    def victory_callback(self, msg: Bool):
        """胜利状态回调"""
        if msg.data:
            self.victory_achieved = True
            self.get_logger().info('🎉 Victory achieved!')
    
    def send_command(self, command: str):
        """发送命令到FSM"""
        cmd_msg = String()
        cmd_msg.data = command
        self.mission_command_pub.publish(cmd_msg)
        self.get_logger().info(f'Sent command: {command}')
    
    def navigate_to_position(self, x: float, y: float, z: float = 0.3, timeout: float = 30.0) -> bool:
        """导航到指定位置"""
        self.get_logger().info(f'🚀 Navigating to position ({x:.2f}, {y:.2f}, {z:.2f})')
        
        command = f"navigate {x} {y} {z}"
        self.send_command(command)
        
        # 等待任务完成
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.current_fsm_state == "VICTORY":
                self.get_logger().info('✅ Navigation completed successfully')
                return True
            elif self.current_fsm_state == "ERROR":
                self.get_logger().error('❌ Navigation failed')
                return False
        
        self.get_logger().error('⏰ Navigation timeout')
        return False
    
    def track_target(self, timeout: float = 60.0) -> bool:
        """跟踪目标直到胜利"""
        self.get_logger().info('🎯 Starting target tracking')
        
        self.victory_achieved = False
        self.send_command("track")
        
        # 等待胜利或超时
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.victory_achieved:
                self.get_logger().info('✅ Target tracking completed successfully')
                return True
            elif self.current_fsm_state == "ERROR":
                self.get_logger().error('❌ Target tracking failed')
                return False
        
        self.get_logger().error('⏰ Target tracking timeout')
        return False
    
    def execute_search_and_track_mission(self, waypoints: List[Tuple[float, float, float]]) -> bool:
        """执行搜索和跟踪任务"""
        self.get_logger().info('🔍 Starting search and track mission')
        
        for i, (x, y, z) in enumerate(waypoints):
            self.get_logger().info(f'📍 Moving to waypoint {i+1}/{len(waypoints)}: ({x:.2f}, {y:.2f}, {z:.2f})')
            
            # 导航到路径点
            if not self.navigate_to_position(x, y, z):
                self.get_logger().error(f'Failed to reach waypoint {i+1}')
                return False
            
            # 在每个路径点尝试跟踪目标
            self.get_logger().info(f'🎯 Searching for target at waypoint {i+1}')
            
            # 短时间跟踪尝试
            success = self.track_target(timeout=10.0)
            if success:
                self.get_logger().info('🎉 Mission completed - target tracked successfully!')
                return True
            
            # 如果没有找到目标，继续下一个路径点
            self.get_logger().info('Target not found, moving to next waypoint')
        
        self.get_logger().warn('⚠️ Mission completed but target was not tracked')
        return False
    
    def stop_all_missions(self):
        """停止所有任务"""
        self.get_logger().info('🛑 Stopping all missions')
        self.send_command("stop")
    
    def reset_system(self):
        """重置系统"""
        self.get_logger().info('🔄 Resetting system')
        self.send_command("reset")
        self.victory_achieved = False
        self.mission_completed = False
    
    def run_demo_mission(self):
        """运行演示任务"""
        self.get_logger().info('🚀 Starting demo mission')
        
        # 定义搜索路径
        search_waypoints = [
            (1.0, 0.0, 0.3),
            (1.0, 1.0, 0.3),
            (0.0, 1.0, 0.3),
            (0.0, 0.0, 0.3),
        ]
        
        try:
            # 重置系统
            self.reset_system()
            time.sleep(2.0)
            
            # 执行搜索和跟踪任务
            success = self.execute_search_and_track_mission(search_waypoints)
            
            if success:
                self.get_logger().info('🎉 Demo mission completed successfully!')
            else:
                self.get_logger().warn('⚠️ Demo mission completed with warnings')
                
        except Exception as e:
            self.get_logger().error(f'❌ Demo mission failed: {e}')
        finally:
            self.stop_all_missions()
    
    def interactive_demo(self):
        """交互式演示"""
        print("\n" + "="*50)
        print("🤖 SIMPLE ROBOT MISSION EXECUTOR")
        print("="*50)
        
        while rclpy.ok():
            print("\nAvailable commands:")
            print("1. Navigate to position")
            print("2. Track target")
            print("3. Search and track mission")
            print("4. Run demo mission")
            print("5. Stop all missions")
            print("6. Reset system")
            print("7. Show current state")
            print("q. Quit")
            print("-"*30)
            
            try:
                choice = input("Enter your choice: ").strip().lower()
                
                if choice == 'q':
                    break
                elif choice == '1':
                    try:
                        x = float(input("Enter X coordinate: "))
                        y = float(input("Enter Y coordinate: "))
                        z = float(input("Enter Z coordinate (default 0.3): ") or "0.3")
                        self.navigate_to_position(x, y, z)
                    except ValueError:
                        print("Invalid coordinates")
                
                elif choice == '2':
                    self.track_target()
                
                elif choice == '3':
                    # 预定义的搜索路径
                    waypoints = [
                        (1.0, 0.0, 0.3),
                        (1.0, 1.0, 0.3),
                        (0.0, 1.0, 0.3),
                        (0.0, 0.0, 0.3),
                    ]
                    self.execute_search_and_track_mission(waypoints)
                
                elif choice == '4':
                    self.run_demo_mission()
                
                elif choice == '5':
                    self.stop_all_missions()
                
                elif choice == '6':
                    self.reset_system()
                
                elif choice == '7':
                    print(f"Current FSM State: {self.current_fsm_state}")
                    print(f"Victory Achieved: {self.victory_achieved}")
                
                else:
                    print("Invalid choice")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        print("\n👋 Mission Executor exiting...")


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()
    
    try:
        # 运行交互式演示
        node.interactive_demo()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
