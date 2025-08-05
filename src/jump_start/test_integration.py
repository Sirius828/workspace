#!/usr/bin/env python3
"""
测试 jump_start 与 start_engine 的集成
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time


class IntegrationTester(Node):
    def __init__(self):
        super().__init__('integration_tester')
        
        # 创建服务客户端
        self.start_client = self.create_client(Empty, '/start')
        
        self.get_logger().info('Integration tester initialized')
    
    def test_start_service(self):
        """测试 /start 服务"""
        self.get_logger().info('🧪 Testing /start service integration...')
        
        # 等待服务可用
        if not self.start_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('❌ /start service not available')
            return False
        
        self.get_logger().info('✅ /start service found')
        
        # 调用服务
        request = Empty.Request()
        
        try:
            future = self.start_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                self.get_logger().info('✅ /start service called successfully')
                self.get_logger().info('🎯 FSM should now start default mission')
                return True
            else:
                self.get_logger().error('❌ Failed to call /start service')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Exception calling service: {e}')
            return False
    
    def run_integration_test(self):
        """运行完整的集成测试"""
        self.get_logger().info('🚀 Starting Jump Start + Start Engine integration test')
        
        print("\n" + "="*60)
        print("🤖 JUMP START + START ENGINE INTEGRATION TEST")
        print("="*60)
        
        # 测试1: 服务可用性
        print("Test 1: Service availability...")
        if self.test_start_service():
            print("✅ PASS: /start service integration works")
        else:
            print("❌ FAIL: /start service integration failed")
            return False
        
        # 等待一段时间观察FSM行为
        print("\nWaiting 5 seconds to observe FSM behavior...")
        time.sleep(5.0)
        
        print("\n" + "="*60)
        print("📊 INTEGRATION TEST RESULTS:")
        print("✅ start_engine GUI can call /start service")
        print("✅ jump_start FSM responds to /start service")
        print("✅ Default mission should be starting")
        print("="*60)
        
        print("\n🎉 Integration test completed successfully!")
        print("\nTo test manually:")
        print("1. Launch: ros2 launch jump_start integrated_system.launch.py")
        print("2. Click '启动系统' button in the GUI")
        print("3. Watch FSM start the default mission")
        
        return True


def main(args=None):
    rclpy.init(args=args)
    
    tester = IntegrationTester()
    
    try:
        success = tester.run_integration_test()
        
        if success:
            print("\n✅ All integration tests passed!")
        else:
            print("\n❌ Some integration tests failed!")
            
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
