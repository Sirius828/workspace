#!/usr/bin/env python3
"""
DrPower电机测试示例

这个脚本演示了如何使用DrPower ROS2 Controllers控制机械臂
包括基本运动、多点轨迹和实时监控
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
import math


class DrPowerTestController(Node):
    def __init__(self):
        super().__init__('drpower_test_controller')
        
        # 创建action客户端
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, 
            '/arm_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # 关节名称
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.current_positions = [0.0] * 6
        
        self.get_logger().info("DrPower测试控制器已启动")
    
    def joint_state_callback(self, msg):
        """关节状态回调"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
    
    def wait_for_servers(self):
        """等待action服务器启动"""
        self.get_logger().info("等待action服务器...")
        self.arm_action_client.wait_for_server()
        self.get_logger().info("所有服务器已就绪")
    
    def send_joint_trajectory(self, positions_list, durations_list):
        """发送关节轨迹
        
        Args:
            positions_list: 位置列表，每个元素是6个关节的位置 [[j1,j2,j3,j4,j5,j6], ...]
            durations_list: 时间列表，到达每个点的时间 [t1, t2, ...]
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        for i, (positions, duration) in enumerate(zip(positions_list, durations_list)):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
            goal_msg.trajectory.points.append(point)
        
        self.get_logger().info(f"发送轨迹：{len(positions_list)}个点")
        future = self.arm_action_client.send_goal_async(goal_msg)
        return future
    
    def test_basic_movement(self):
        """测试基本运动"""
        self.get_logger().info("=== 测试1: 基本运动 ===")
        
        # 回到零位
        positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        future = self.send_joint_trajectory([positions], [3.0])
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            self.get_logger().info("✓ 回零位完成")
        else:
            self.get_logger().error("✗ 回零位失败")
            return False
        
        time.sleep(1)
        
        # 移动到测试位置
        positions = [math.pi/4, -math.pi/3, math.pi/2, 0.0, math.pi/3, 0.0]
        future = self.send_joint_trajectory([positions], [5.0])
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            self.get_logger().info("✓ 移动到测试位置完成")
            return True
        else:
            self.get_logger().error("✗ 移动到测试位置失败")
            return False
    
    def test_multi_point_trajectory(self):
        """测试多点轨迹"""
        self.get_logger().info("=== 测试2: 多点轨迹 ===")
        
        # 定义多个关键点
        waypoints = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],           # 起点
            [math.pi/6, -math.pi/6, math.pi/4, 0.0, math.pi/6, 0.0],  # 点1
            [math.pi/3, -math.pi/3, math.pi/2, 0.0, math.pi/3, 0.0],  # 点2
            [math.pi/4, -math.pi/4, math.pi/3, 0.0, math.pi/4, 0.0],  # 点3
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],           # 回到起点
        ]
        
        # 定义时间点
        times = [2.0, 4.0, 6.0, 8.0, 10.0]
        
        future = self.send_joint_trajectory(waypoints, times)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            self.get_logger().info("✓ 多点轨迹执行完成")
            return True
        else:
            self.get_logger().error("✗ 多点轨迹执行失败")
            return False
    
    def test_circular_motion(self):
        """测试圆形运动（仅前三个关节）"""
        self.get_logger().info("=== 测试3: 圆形运动 ===")
        
        # 生成圆形轨迹
        waypoints = []
        times = []
        
        for i in range(13):  # 0到2π，12个点加回到起点
            angle = i * 2 * math.pi / 12
            
            # 简单的圆形运动：关节1和2形成圆形
            j1 = 0.3 * math.cos(angle)
            j2 = 0.3 * math.sin(angle)
            j3 = 0.2 * math.sin(2 * angle)  # 添加一些变化
            
            positions = [j1, j2, j3, 0.0, 0.0, 0.0]
            waypoints.append(positions)
            times.append((i + 1) * 1.0)  # 每个点间隔1秒
        
        future = self.send_joint_trajectory(waypoints, times)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            self.get_logger().info("✓ 圆形运动执行完成")
            return True
        else:
            self.get_logger().error("✗ 圆形运动执行失败")
            return False
    
    def monitor_joint_states(self, duration=5.0):
        """监控关节状态"""
        self.get_logger().info(f"=== 监控关节状态 {duration}秒 ===")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            # 打印当前关节位置
            pos_str = ", ".join([f"{pos:.3f}" for pos in self.current_positions])
            self.get_logger().info(f"关节位置: [{pos_str}]")
            time.sleep(1.0)
    
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info("开始DrPower电机测试...")
        
        # 等待服务器
        self.wait_for_servers()
        
        # 监控初始状态
        self.monitor_joint_states(3.0)
        
        # 运行测试
        tests = [
            self.test_basic_movement,
            self.test_multi_point_trajectory,
            self.test_circular_motion
        ]
        
        passed = 0
        for test in tests:
            if test():
                passed += 1
            time.sleep(2)  # 测试间隔
        
        # 最后回零
        self.get_logger().info("=== 回零位结束测试 ===")
        positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        future = self.send_joint_trajectory([positions], [5.0])
        rclpy.spin_until_future_complete(self, future)
        
        # 结果统计
        self.get_logger().info(f"测试完成！通过 {passed}/{len(tests)} 项测试")
        return passed == len(tests)


def main():
    rclpy.init()
    
    try:
        controller = DrPowerTestController()
        success = controller.run_all_tests()
        
        if success:
            controller.get_logger().info("🎉 所有测试通过！DrPower系统工作正常")
        else:
            controller.get_logger().warn("⚠️ 部分测试失败，请检查系统状态")
            
    except KeyboardInterrupt:
        print("测试被用户中断")
    except Exception as e:
        print(f"测试过程中出现错误: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
