#!/usr/bin/env python3
"""
测试脚本：调用start服务
"""

import rclcpp
from rclcpp.node import Node
from std_srvs.srv import Empty

class ServiceCaller(Node):
    def __init__(self):
        super().__init__('service_caller')
        self.client = self.create_client(Empty, 'start')
        
    def call_start_service(self):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('服务 /start 不可用')
            return False
            
        request = Empty.Request()
        future = self.client.call_async(request)
        
        rclcpp.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('✅ 成功调用 start 服务')
            return True
        else:
            self.get_logger().error('❌ 调用 start 服务失败')
            return False

def main():
    rclcpp.init()
    
    caller = ServiceCaller()
    success = caller.call_start_service()
    
    rclcpp.shutdown()
    
    if success:
        print("任务已启动！")
    else:
        print("任务启动失败！")

if __name__ == '__main__':
    main()
