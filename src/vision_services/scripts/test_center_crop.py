#!/usr/bin/env python3
"""
测试脚本：验证中心裁切功能
"""

import rclpy
from rclpy.node import Node
from vision_services_msgs.srv import A4Loop
import time


class CropTestClient(Node):
    """中心裁切测试客户端"""
    
    def __init__(self):
        super().__init__('crop_test_client')
        
        # 创建服务客户端
        self.cli = self.create_client(A4Loop, 'vision_services/a4_loop')
        
        # 等待服务可用
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 A4 检测服务可用...')
        
        self.get_logger().info('A4 检测服务已连接')
    
    def test_a4_detection(self):
        """测试 A4 检测"""
        req = A4Loop.Request()
        
        self.get_logger().info('调用 A4 检测服务（带中心裁切）...')
        
        try:
            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                response = future.result()
                self.get_logger().info(f'检测成功: {response.success}')
                
                if response.success:
                    self.get_logger().info(f'检测到的中心点: ({response.center.x:.2f}, {response.center.y:.2f})')
                    self.get_logger().info(f'检测到角点数量: {len(response.corners)}')
                    
                    for i, corner in enumerate(response.corners):
                        self.get_logger().info(f'角点 {i+1}: ({corner.x:.2f}, {corner.y:.2f})')
                else:
                    self.get_logger().info('未检测到 A4 框')
            else:
                self.get_logger().error('服务调用失败')
                
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {e}')


def main():
    rclpy.init()
    
    client = CropTestClient()
    
    # 等待一段时间让图像流稳定
    time.sleep(2)
    
    # 执行测试
    client.test_a4_detection()
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
