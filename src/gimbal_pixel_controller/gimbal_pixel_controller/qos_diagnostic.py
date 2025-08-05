#!/usr/bin/env python3
"""
QoS 诊断工具
检查云台控制系统中各个话题的QoS兼容性
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import subprocess
import re

class QoSDiagnosticTool(Node):
    def __init__(self):
        super().__init__('qos_diagnostic_tool')
        
        # 要检查的话题列表
        self.topics_to_check = [
            '/target_position_pixel',
            '/cmd_gimbal',
            '/gimbal_status',
            '/vision_services/detection_result'
        ]
        
        self.get_logger().info('QoS诊断工具启动')
        self.check_all_topics()
    
    def get_topic_info(self, topic_name):
        """获取话题的QoS信息"""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'info', topic_name, '-v'], 
                capture_output=True, 
                text=True, 
                timeout=5
            )
            
            if result.returncode == 0:
                return result.stdout
            else:
                return None
        except subprocess.TimeoutExpired:
            return None
        except Exception as e:
            self.get_logger().error(f'获取话题信息失败: {e}')
            return None
    
    def parse_qos_info(self, topic_info):
        """解析QoS信息"""
        if not topic_info:
            return None
        
        publishers = []
        subscribers = []
        
        # 解析发布者和订阅者的QoS设置
        current_section = None
        current_node = None
        current_qos = {}
        
        for line in topic_info.split('\n'):
            line = line.strip()
            
            if 'Publisher count:' in line:
                current_section = 'publishers'
            elif 'Subscription count:' in line:
                current_section = 'subscribers'
            elif line.startswith('Node name:'):
                if current_qos and current_node:
                    if current_section == 'publishers':
                        publishers.append({'node': current_node, 'qos': current_qos.copy()})
                    elif current_section == 'subscribers':
                        subscribers.append({'node': current_node, 'qos': current_qos.copy()})
                
                current_node = line.split(':', 1)[1].strip()
                current_qos = {}
            elif 'Reliability:' in line:
                current_qos['reliability'] = line.split(':', 1)[1].strip()
            elif 'Durability:' in line:
                current_qos['durability'] = line.split(':', 1)[1].strip()
            elif 'Depth:' in line:
                current_qos['depth'] = line.split(':', 1)[1].strip()
        
        # 添加最后一个节点
        if current_qos and current_node:
            if current_section == 'publishers':
                publishers.append({'node': current_node, 'qos': current_qos.copy()})
            elif current_section == 'subscribers':
                subscribers.append({'node': current_node, 'qos': current_qos.copy()})
        
        return {
            'publishers': publishers,
            'subscribers': subscribers
        }
    
    def check_qos_compatibility(self, publishers, subscribers):
        """检查QoS兼容性"""
        incompatible_pairs = []
        
        for pub in publishers:
            for sub in subscribers:
                pub_reliability = pub['qos'].get('reliability', 'UNKNOWN')
                sub_reliability = sub['qos'].get('reliability', 'UNKNOWN')
                
                # RELIABLE发布者可以与任何订阅者兼容
                # BEST_EFFORT发布者只能与BEST_EFFORT订阅者兼容
                if (pub_reliability == 'BEST_EFFORT' and 
                    sub_reliability == 'RELIABLE'):
                    incompatible_pairs.append({
                        'publisher': pub['node'],
                        'subscriber': sub['node'],
                        'issue': f'发布者使用{pub_reliability}，订阅者需要{sub_reliability}'
                    })
        
        return incompatible_pairs
    
    def check_topic(self, topic_name):
        """检查单个话题"""
        self.get_logger().info(f'检查话题: {topic_name}')
        
        topic_info = self.get_topic_info(topic_name)
        if not topic_info:
            self.get_logger().warn(f'无法获取话题 {topic_name} 的信息')
            return
        
        qos_data = self.parse_qos_info(topic_info)
        if not qos_data:
            self.get_logger().warn(f'无法解析话题 {topic_name} 的QoS信息')
            return
        
        publishers = qos_data['publishers']
        subscribers = qos_data['subscribers']
        
        self.get_logger().info(f'  发布者数量: {len(publishers)}')
        self.get_logger().info(f'  订阅者数量: {len(subscribers)}')
        
        # 显示发布者信息
        for pub in publishers:
            reliability = pub['qos'].get('reliability', 'UNKNOWN')
            durability = pub['qos'].get('durability', 'UNKNOWN')
            self.get_logger().info(f'    发布者 {pub["node"]}: {reliability}/{durability}')
        
        # 显示订阅者信息
        for sub in subscribers:
            reliability = sub['qos'].get('reliability', 'UNKNOWN')
            durability = sub['qos'].get('durability', 'UNKNOWN')
            self.get_logger().info(f'    订阅者 {sub["node"]}: {reliability}/{durability}')
        
        # 检查兼容性
        incompatible = self.check_qos_compatibility(publishers, subscribers)
        if incompatible:
            self.get_logger().warn(f'  发现 {len(incompatible)} 个不兼容配置:')
            for issue in incompatible:
                self.get_logger().warn(f'    {issue["publisher"]} -> {issue["subscriber"]}: {issue["issue"]}')
        else:
            self.get_logger().info(f'  ✓ 所有QoS配置兼容')
    
    def check_all_topics(self):
        """检查所有话题"""
        self.get_logger().info('开始QoS兼容性检查...')
        
        for topic in self.topics_to_check:
            self.check_topic(topic)
            print()  # 添加空行分隔
        
        self.get_logger().info('QoS检查完成')

def main(args=None):
    rclpy.init(args=args)
    
    diagnostic_tool = QoSDiagnosticTool()
    
    # 不需要spin，工具运行一次后退出
    diagnostic_tool.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
