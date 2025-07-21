#!/usr/bin/env python3
"""
DrPower ROS2控制器高级测试脚本
演示完整的功能和集成测试
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import time
import threading
import math
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion
# MoveIt2消息类型 (可选，如果系统未安装MoveIt2则跳过)
try:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (
        MotionPlanRequest, 
        PlanningOptions, 
        Constraints,
        JointConstraint,
        PositionConstraint,
        BoundingVolume
    )
    from shape_msgs.msg import SolidPrimitive
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    print("MoveIt2消息包未安装，跳过相关功能")

class DrPowerAdvancedTest(Node):
    """DrPower高级测试节点"""
    
    def __init__(self):
        super().__init__('drpower_advanced_test')
        
        # 参数配置
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]
        
        # 发布器
        self.position_publisher = self.create_publisher(
            Float64MultiArray,
            '/drpower_arm_controller/commands',
            10
        )
        
        # 订阅器
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Action客户端
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/drpower_arm_controller/follow_joint_trajectory'
        )
        
        # 状态变量
        self.current_joint_states = None
        self.test_results = {}
        
        # 等待服务就绪
        self.get_logger().info("等待控制器就绪...")
        time.sleep(2.0)
        
    def joint_state_callback(self, msg):
        """关节状态回调"""
        self.current_joint_states = msg
        
    def wait_for_joint_states(self, timeout=5.0):
        """等待关节状态"""
        start_time = time.time()
        while self.current_joint_states is None:
            if time.time() - start_time > timeout:
                return False
            time.sleep(0.1)
        return True
        
    def test_basic_position_control(self):
        """测试基本位置控制"""
        self.get_logger().info("=== 测试基本位置控制 ===")
        
        try:
            # 测试位置序列
            test_positions = [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 零位
                [0.5, 0.3, -0.2, 0.0, 0.0, 0.0],  # 测试位置1
                [1.0, 0.5, -0.5, 0.5, 0.3, 0.0],  # 测试位置2
                [-0.5, -0.3, 0.2, -0.5, -0.3, 0.0],  # 测试位置3
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 返回零位
            ]
            
            success_count = 0
            for i, position in enumerate(test_positions):
                self.get_logger().info(f"发送位置命令 {i+1}: {position}")
                
                # 发送位置命令
                msg = Float64MultiArray()
                msg.data = position
                self.position_publisher.publish(msg)
                
                # 等待执行
                time.sleep(3.0)
                
                # 检查是否到达目标位置
                if self.current_joint_states:
                    current_pos = list(self.current_joint_states.position)
                    error = sum(abs(c - t) for c, t in zip(current_pos, position))
                    if error < 0.1:  # 允许0.1弧度误差
                        success_count += 1
                        self.get_logger().info(f"位置 {i+1} 到达成功")
                    else:
                        self.get_logger().warn(f"位置 {i+1} 到达失败，误差: {error}")
                
            success_rate = success_count / len(test_positions)
            self.test_results['basic_position_control'] = {
                'success_rate': success_rate,
                'total_tests': len(test_positions),
                'successful_tests': success_count
            }
            
            self.get_logger().info(f"基本位置控制测试完成，成功率: {success_rate:.2%}")
            return success_rate > 0.8
            
        except Exception as e:
            self.get_logger().error(f"基本位置控制测试失败: {e}")
            return False
            
    def test_trajectory_following(self):
        """测试轨迹跟踪"""
        self.get_logger().info("=== 测试轨迹跟踪 ===")
        
        try:
            # 等待action服务器
            if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("轨迹控制器服务器未就绪")
                return False
                
            # 创建轨迹
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = JointTrajectory()
            goal.trajectory.joint_names = self.joint_names
            
            # 轨迹点
            points = [
                ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0),
                ([0.5, 0.3, -0.2, 0.0, 0.0, 0.0], 2.0),
                ([1.0, 0.5, -0.5, 0.5, 0.3, 0.0], 4.0),
                ([0.5, 0.3, -0.2, 0.0, 0.0, 0.0], 6.0),
                ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 8.0),
            ]
            
            for positions, time_sec in points:
                point = JointTrajectoryPoint()
                point.positions = positions
                point.time_from_start = Duration(seconds=time_sec).to_msg()
                goal.trajectory.points.append(point)
                
            # 发送轨迹
            self.get_logger().info("发送轨迹跟踪命令...")
            future = self.trajectory_client.send_goal_async(goal)
            
            # 等待结果
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            
            if future.result():
                goal_handle = future.result()
                if goal_handle.accepted:
                    self.get_logger().info("轨迹目标被接受")
                    
                    # 等待执行完成
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
                    
                    if result_future.result():
                        result = result_future.result().result
                        success = result.error_code == 0
                        self.test_results['trajectory_following'] = {
                            'success': success,
                            'error_code': result.error_code
                        }
                        self.get_logger().info(f"轨迹跟踪完成，成功: {success}")
                        return success
                        
            self.get_logger().error("轨迹跟踪失败")
            return False
            
        except Exception as e:
            self.get_logger().error(f"轨迹跟踪测试失败: {e}")
            return False
            
    def test_safety_limits(self):
        """测试安全限制"""
        self.get_logger().info("=== 测试安全限制 ===")
        
        try:
            # 测试超出限制的位置
            extreme_positions = [
                [4.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 超出joint1限制
                [0.0, 2.0, 0.0, 0.0, 0.0, 0.0],  # 超出joint2限制
                [0.0, 0.0, 2.0, 0.0, 0.0, 0.0],  # 超出joint3限制
            ]
            
            safety_working = 0
            for i, position in enumerate(extreme_positions):
                self.get_logger().info(f"测试极限位置 {i+1}: {position}")
                
                # 记录当前位置
                initial_pos = None
                if self.current_joint_states:
                    initial_pos = list(self.current_joint_states.position)
                
                # 发送极限位置命令
                msg = Float64MultiArray()
                msg.data = position
                self.position_publisher.publish(msg)
                
                # 等待响应
                time.sleep(2.0)
                
                # 检查是否被安全限制阻止
                if self.current_joint_states and initial_pos:
                    current_pos = list(self.current_joint_states.position)
                    movement = sum(abs(c - i) for c, i in zip(current_pos, initial_pos))
                    
                    if movement < 0.1:  # 几乎没有移动，说明被安全限制
                        safety_working += 1
                        self.get_logger().info(f"安全限制 {i+1} 工作正常")
                    else:
                        self.get_logger().warn(f"安全限制 {i+1} 可能失效")
                        
            success_rate = safety_working / len(extreme_positions)
            self.test_results['safety_limits'] = {
                'success_rate': success_rate,
                'total_tests': len(extreme_positions),
                'working_limits': safety_working
            }
            
            self.get_logger().info(f"安全限制测试完成，成功率: {success_rate:.2%}")
            return success_rate > 0.5
            
        except Exception as e:
            self.get_logger().error(f"安全限制测试失败: {e}")
            return False
            
    def test_multi_motor_coordination(self):
        """测试多电机协调"""
        self.get_logger().info("=== 测试多电机协调 ===")
        
        try:
            # 测试协调运动模式
            coordination_tests = [
                # 测试1: 前三个关节同时运动
                ([0.5, 0.5, 0.5, 0.0, 0.0, 0.0], "前三关节协调"),
                # 测试2: 后三个关节同时运动  
                ([0.0, 0.0, 0.0, 0.5, 0.5, 0.5], "后三关节协调"),
                # 测试3: 全部关节协调运动
                ([0.3, 0.3, 0.3, 0.3, 0.3, 0.3], "全关节协调"),
                # 测试4: 返回零位
                ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "返回零位"),
            ]
            
            coordination_success = 0
            for position, description in coordination_tests:
                self.get_logger().info(f"测试 {description}: {position}")
                
                # 记录开始时间
                start_time = time.time()
                
                # 发送协调命令
                msg = Float64MultiArray()
                msg.data = position
                self.position_publisher.publish(msg)
                
                # 监控运动过程
                motion_time = 0.0
                while motion_time < 3.0:
                    time.sleep(0.1)
                    motion_time = time.time() - start_time
                    
                    if self.current_joint_states:
                        # 检查运动同步性
                        current_pos = list(self.current_joint_states.position)
                        # 这里可以添加更复杂的同步性检查逻辑
                        
                # 检查最终位置
                if self.current_joint_states:
                    current_pos = list(self.current_joint_states.position)
                    error = sum(abs(c - t) for c, t in zip(current_pos, position))
                    if error < 0.2:
                        coordination_success += 1
                        self.get_logger().info(f"{description} 成功")
                    else:
                        self.get_logger().warn(f"{description} 失败，误差: {error}")
                        
            success_rate = coordination_success / len(coordination_tests)
            self.test_results['multi_motor_coordination'] = {
                'success_rate': success_rate,
                'total_tests': len(coordination_tests),
                'successful_tests': coordination_success
            }
            
            self.get_logger().info(f"多电机协调测试完成，成功率: {success_rate:.2%}")
            return success_rate > 0.75
            
        except Exception as e:
            self.get_logger().error(f"多电机协调测试失败: {e}")
            return False
            
    def test_real_time_performance(self):
        """测试实时性能"""
        self.get_logger().info("=== 测试实时性能 ===")
        
        try:
            # 性能测试参数
            test_duration = 10.0  # 秒
            command_frequency = 50.0  # Hz
            
            # 记录性能数据
            command_times = []
            state_times = []
            start_time = time.time()
            
            def state_callback_performance(msg):
                state_times.append(time.time())
                
            # 临时订阅器用于性能测试
            perf_sub = self.create_subscription(
                JointState,
                '/joint_states',
                state_callback_performance,
                10
            )
            
            # 发送高频命令
            command_count = 0
            while time.time() - start_time < test_duration:
                # 生成正弦波位置命令
                t = time.time() - start_time
                amplitude = 0.3
                frequency = 0.5
                position = [
                    amplitude * math.sin(2 * math.pi * frequency * t),
                    amplitude * math.cos(2 * math.pi * frequency * t),
                    0.0, 0.0, 0.0, 0.0
                ]
                
                # 发送命令
                msg = Float64MultiArray()
                msg.data = position
                self.position_publisher.publish(msg)
                command_times.append(time.time())
                command_count += 1
                
                # 控制频率
                time.sleep(1.0 / command_frequency)
                
            # 清理临时订阅器
            self.destroy_subscription(perf_sub)
            
            # 分析性能
            actual_command_freq = len(command_times) / test_duration
            actual_state_freq = len(state_times) / test_duration
            
            # 计算延迟统计
            latencies = []
            for i in range(min(len(command_times), len(state_times))):
                if i < len(state_times):
                    latency = state_times[i] - command_times[i]
                    if latency > 0:  # 只考虑正延迟
                        latencies.append(latency)
                        
            avg_latency = sum(latencies) / len(latencies) if latencies else 0
            max_latency = max(latencies) if latencies else 0
            
            self.test_results['real_time_performance'] = {
                'command_frequency': actual_command_freq,
                'state_frequency': actual_state_freq,
                'average_latency': avg_latency,
                'max_latency': max_latency,
                'test_duration': test_duration
            }
            
            self.get_logger().info(f"实时性能测试完成:")
            self.get_logger().info(f"  命令频率: {actual_command_freq:.1f} Hz")
            self.get_logger().info(f"  状态频率: {actual_state_freq:.1f} Hz")
            self.get_logger().info(f"  平均延迟: {avg_latency*1000:.1f} ms")
            self.get_logger().info(f"  最大延迟: {max_latency*1000:.1f} ms")
            
            # 性能合格标准
            freq_ok = actual_command_freq > 40.0 and actual_state_freq > 40.0
            latency_ok = avg_latency < 0.05  # 50ms
            
            return freq_ok and latency_ok
            
        except Exception as e:
            self.get_logger().error(f"实时性能测试失败: {e}")
            return False
            
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info("开始DrPower ROS2控制器综合测试...")
        
        # 等待系统就绪
        if not self.wait_for_joint_states():
            self.get_logger().error("无法获取关节状态，测试终止")
            return
            
        # 测试列表
        tests = [
            ("基本位置控制", self.test_basic_position_control),
            ("轨迹跟踪", self.test_trajectory_following),
            ("安全限制", self.test_safety_limits),
            ("多电机协调", self.test_multi_motor_coordination),
            ("实时性能", self.test_real_time_performance),
        ]
        
        # 执行测试
        passed_tests = 0
        total_tests = len(tests)
        
        for test_name, test_func in tests:
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"开始测试: {test_name}")
            self.get_logger().info(f"{'='*50}")
            
            try:
                result = test_func()
                if result:
                    passed_tests += 1
                    self.get_logger().info(f"✅ {test_name} - 通过")
                else:
                    self.get_logger().error(f"❌ {test_name} - 失败")
            except Exception as e:
                self.get_logger().error(f"❌ {test_name} - 异常: {e}")
                
            # 测试间隔
            time.sleep(1.0)
            
        # 生成测试报告
        self.generate_test_report(passed_tests, total_tests)
        
    def generate_test_report(self, passed, total):
        """生成测试报告"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("DrPower ROS2控制器测试报告")
        self.get_logger().info(f"{'='*60}")
        self.get_logger().info(f"总测试数: {total}")
        self.get_logger().info(f"通过测试: {passed}")
        self.get_logger().info(f"失败测试: {total - passed}")
        self.get_logger().info(f"成功率: {passed/total:.1%}")
        
        # 详细结果
        for test_name, result in self.test_results.items():
            self.get_logger().info(f"\n{test_name}:")
            if isinstance(result, dict):
                for key, value in result.items():
                    self.get_logger().info(f"  {key}: {value}")
                    
        self.get_logger().info(f"{'='*60}")
        
        if passed == total:
            self.get_logger().info("🎉 所有测试通过！DrPower ROS2控制器工作正常。")
        else:
            self.get_logger().warn(f"⚠️  {total-passed} 个测试失败，请检查系统配置。")


def main():
    """主函数"""
    rclpy.init()
    
    # 创建测试节点
    test_node = DrPowerAdvancedTest()
    
    try:
        # 运行所有测试
        test_node.run_all_tests()
        
        # 保持节点运行一段时间以完成所有操作
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        test_node.get_logger().info("测试被用户中断")
    except Exception as e:
        test_node.get_logger().error(f"测试过程中发生错误: {e}")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
