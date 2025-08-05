#!/usr/bin/env python3

"""
行为树演示脚本

这个脚本演示了如何使用行为树系统执行不同的机器人任务。
包含多个预设任务场景，可用于测试和演示。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import sys

# Action message types
from robot_behavior_tree.action import RobotMission, SearchAndTrack, PatrolMission

# Standard message types
from geometry_msgs.msg import Point
from std_msgs.msg import String

class MissionDemo(Node):
    def __init__(self):
        super().__init__('mission_demo')
        
        # Action clients
        self.robot_mission_client = ActionClient(self, RobotMission, '/robot_mission')
        self.search_track_client = ActionClient(self, SearchAndTrack, '/search_and_track')
        self.patrol_client = ActionClient(self, PatrolMission, '/patrol_mission')
        
        # Command publisher for direct behavior tree control
        self.bt_command_pub = self.create_publisher(String, '/behavior_tree/command', 10)
        
        self.get_logger().info("Mission Demo Node initialized")
    
    def wait_for_servers(self, timeout_sec=10.0):
        """等待Action服务器就绪"""
        self.get_logger().info("Waiting for action servers...")
        
        servers = [
            (self.robot_mission_client, "RobotMission"),
            (self.search_track_client, "SearchAndTrack"), 
            (self.patrol_client, "PatrolMission")
        ]
        
        for client, name in servers:
            if not client.wait_for_server(timeout_sec=timeout_sec):
                self.get_logger().warn(f"{name} action server not available")
            else:
                self.get_logger().info(f"{name} action server ready")
    
    def send_bt_command(self, command: str):
        """发送行为树命令"""
        msg = String()
        msg.data = command
        self.bt_command_pub.publish(msg)
        self.get_logger().info(f"Sent BT command: {command}")
    
    def demo_search_and_track(self):
        """演示搜索和追踪任务"""
        self.get_logger().info("🔍 Starting Search and Track Demo")
        
        if not self.search_track_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("SearchAndTrack action server not available")
            return False
        
        # Create goal
        goal = SearchAndTrack.Goal()
        goal.target_class = "ferrari"
        goal.search_center = Point(x=0.0, y=0.0, z=0.0)
        goal.search_radius = 2.0
        goal.detection_confidence = 0.8
        goal.max_track_time = 30.0
        
        # Send goal
        self.get_logger().info("Sending search and track goal...")
        send_goal_future = self.search_track_client.send_goal_async(
            goal, feedback_callback=self.search_track_feedback_callback)
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False
        
        self.get_logger().info("Goal accepted, waiting for result...")
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        self.get_logger().info(f"Search and Track completed: target_found={result.target_found}")
        
        return result.target_found
    
    def search_track_feedback_callback(self, feedback_msg):
        """搜索追踪反馈回调"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Search status: {feedback.search_status}, "
                              f"Target in view: {feedback.target_in_view}")
    
    def demo_patrol_mission(self):
        """演示巡逻任务"""
        self.get_logger().info("🚶 Starting Patrol Mission Demo")
        
        if not self.patrol_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("PatrolMission action server not available")
            return False
        
        # Create patrol points (square pattern)
        patrol_points = [
            Point(x=1.0, y=0.0, z=0.0),
            Point(x=1.0, y=1.0, z=0.0),
            Point(x=0.0, y=1.0, z=0.0),
            Point(x=0.0, y=0.0, z=0.0),
        ]
        
        # Create goal
        goal = PatrolMission.Goal()
        goal.patrol_points = patrol_points
        goal.loop_patrol = True
        goal.patrol_speed = 0.3
        goal.wait_time_at_point = 2.0
        goal.enable_detection = True
        goal.detection_target = "ferrari"
        
        # Send goal
        self.get_logger().info("Sending patrol mission goal...")
        send_goal_future = self.patrol_client.send_goal_async(
            goal, feedback_callback=self.patrol_feedback_callback)
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Patrol goal rejected")
            return False
        
        self.get_logger().info("Patrol goal accepted, executing...")
        
        # Let it run for a while, then cancel
        time.sleep(20)  # Run for 20 seconds
        
        self.get_logger().info("Cancelling patrol mission...")
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)
        
        return True
    
    def patrol_feedback_callback(self, feedback_msg):
        """巡逻反馈回调"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Patrol: Point {feedback.current_point_index}, "
                              f"Distance to next: {feedback.distance_to_next_point:.2f}m")
    
    def demo_direct_bt_control(self):
        """演示直接行为树控制"""
        self.get_logger().info("🎮 Demonstrating direct BehaviorTree control")
        
        commands = [
            ("start", "Starting behavior tree"),
            ("stop", "Stopping behavior tree"),
            ("reset", "Resetting behavior tree"),
            ("start", "Restarting behavior tree")
        ]
        
        for command, description in commands:
            self.get_logger().info(description)
            self.send_bt_command(command)
            time.sleep(3)  # Wait between commands
    
    def run_demo_sequence(self):
        """运行完整的演示序列"""
        self.get_logger().info("🚀 Starting Robot Behavior Tree Demo Sequence")
        
        demos = [
            ("Direct BehaviorTree Control", self.demo_direct_bt_control),
            ("Search and Track Mission", self.demo_search_and_track),
            ("Patrol Mission", self.demo_patrol_mission),
        ]
        
        for demo_name, demo_func in demos:
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"Starting: {demo_name}")
            self.get_logger().info('='*50)
            
            try:
                success = demo_func()
                if success:
                    self.get_logger().info(f"✅ {demo_name} completed successfully")
                else:
                    self.get_logger().warn(f"⚠️ {demo_name} completed with warnings")
            except Exception as e:
                self.get_logger().error(f"❌ {demo_name} failed: {e}")
            
            # Wait between demos
            self.get_logger().info("Waiting before next demo...")
            time.sleep(5)
        
        self.get_logger().info("\n🎉 All demos completed!")
    
    def interactive_demo(self):
        """交互式演示菜单"""
        while rclpy.ok():
            print("\n" + "="*50)
            print("🤖 ROBOT BEHAVIOR TREE DEMO MENU")
            print("="*50)
            print("1. Search and Track Demo")
            print("2. Patrol Mission Demo") 
            print("3. Direct BT Control Demo")
            print("4. Run All Demos")
            print("5. Start BehaviorTree")
            print("6. Stop BehaviorTree")
            print("7. Reset BehaviorTree")
            print("q. Quit")
            print("-"*50)
            
            try:
                choice = input("Enter your choice: ").strip().lower()
                
                if choice == 'q':
                    break
                elif choice == '1':
                    self.demo_search_and_track()
                elif choice == '2':
                    self.demo_patrol_mission()
                elif choice == '3':
                    self.demo_direct_bt_control()
                elif choice == '4':
                    self.run_demo_sequence()
                elif choice == '5':
                    self.send_bt_command("start")
                elif choice == '6':
                    self.send_bt_command("stop")
                elif choice == '7':
                    self.send_bt_command("reset")
                else:
                    print("Invalid choice, please try again.")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        print("\n👋 Demo exiting...")

def main():
    rclpy.init()
    
    demo_node = MissionDemo()
    
    # Wait for action servers
    demo_node.wait_for_servers()
    
    try:
        if len(sys.argv) > 1:
            if sys.argv[1] == "--auto":
                # Run automatic demo sequence
                demo_node.run_demo_sequence()
            elif sys.argv[1] == "--search":
                # Run only search and track demo
                demo_node.demo_search_and_track()
            elif sys.argv[1] == "--patrol":
                # Run only patrol demo
                demo_node.demo_patrol_mission()
        else:
            # Run interactive demo
            demo_node.interactive_demo()
            
    except KeyboardInterrupt:
        pass
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
