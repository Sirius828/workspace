#!/usr/bin/env python3
"""
模糊PID控制器参数调试工具
用于实时调整参数以减少抖动
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
import time

class FuzzyPIDTuner(Node):
    def __init__(self):
        super().__init__('fuzzy_pid_tuner')
        
        # 创建服务客户端
        self.param_client = self.create_client(
            SetParameters,
            '/gimbal_pixel_controller/set_parameters'
        )
        
        # 等待服务可用
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待参数服务...')
        
        self.get_logger().info('模糊PID参数调试工具已启动')
        
    def set_parameter(self, name, value, param_type='double'):
        """设置参数"""
        request = SetParameters.Request()
        
        param = Parameter()
        param.name = name
        
        param_val = ParameterValue()
        if param_type == 'double':
            param_val.type = ParameterValue.PARAMETER_DOUBLE
            param_val.double_value = float(value)
        elif param_type == 'integer':
            param_val.type = ParameterValue.PARAMETER_INTEGER
            param_val.integer_value = int(value)
        elif param_type == 'double_array':
            param_val.type = ParameterValue.PARAMETER_DOUBLE_ARRAY
            param_val.double_array_value = [float(x) for x in value]
        
        param.value = param_val
        request.parameters = [param]
        
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().results[0].successful:
            self.get_logger().info(f'成功设置参数 {name} = {value}')
            return True
        else:
            self.get_logger().error(f'设置参数 {name} 失败: {future.result().results[0].reason}')
            return False
    
    def interactive_tuning(self):
        """交互式参数调整"""
        print("\n=== 模糊PID参数调试工具 ===")
        print("用于减少静止抖动的参数调整")
        print("=============================")
        
        while True:
            print("\n参数调整选项:")
            print("1. 基础PID参数")
            print("2. 死区和滤波参数")
            print("3. 模糊规则参数")
            print("4. 快速抗抖动设置")
            print("5. 退出")
            
            choice = input("\n请选择 (1-5): ")
            
            try:
                if choice == '1':
                    self.tune_basic_pid()
                elif choice == '2':
                    self.tune_filter_params()
                elif choice == '3':
                    self.tune_fuzzy_rules()
                elif choice == '4':
                    self.quick_anti_shake_setup()
                elif choice == '5':
                    print("退出调试工具")
                    break
                else:
                    print("无效选择")
            except KeyboardInterrupt:
                print("\n退出调试工具")
                break
            except Exception as e:
                print(f"错误: {e}")
    
    def tune_basic_pid(self):
        """调整基础PID参数"""
        print("\n--- 基础PID参数调整 ---")
        print("当前推荐值（抗抖动）:")
        print("yaw_kp: 0.03-0.08")
        print("pitch_kp: 0.03-0.08")
        print("kd: 0.005-0.015")
        
        params = {
            'yaw_kp': '偏航比例增益',
            'yaw_kd': '偏航微分增益',
            'pitch_kp': '俯仰比例增益',
            'pitch_kd': '俯仰微分增益'
        }
        
        for param_name, description in params.items():
            value = input(f"输入 {description} ({param_name}) [回车跳过]: ")
            if value.strip():
                try:
                    self.set_parameter(param_name, float(value))
                except ValueError:
                    print("无效数值")
    
    def tune_filter_params(self):
        """调整滤波和死区参数"""
        print("\n--- 滤波和死区参数调整 ---")
        print("当前推荐值（抗抖动）:")
        print("deadzone_pixel: 2.0-5.0 (像素)")
        print("filter_alpha: 0.6-0.8 (越大越平滑)")
        print("min_movement_threshold: 0.0005-0.002 (弧度)")
        print("min_confidence: 0.8-0.9")
        
        params = {
            'deadzone_pixel': '像素死区',
            'filter_alpha': '滤波器系数',
            'min_movement_threshold': '最小运动阈值',
            'min_confidence': '最小置信度',
            'max_angle_change': '最大角度变化'
        }
        
        for param_name, description in params.items():
            value = input(f"输入 {description} ({param_name}) [回车跳过]: ")
            if value.strip():
                try:
                    self.set_parameter(param_name, float(value))
                except ValueError:
                    print("无效数值")
    
    def tune_fuzzy_rules(self):
        """调整模糊规则参数"""
        print("\n--- 模糊规则参数调整 ---")
        print("调整误差范围阈值:")
        
        params = {
            'fuzzy_error_small': '小误差阈值 (推荐: 0.03-0.08)',
            'fuzzy_error_medium': '中等误差阈值 (推荐: 0.15-0.25)',
            'fuzzy_derror_small': '小误差变化率阈值 (推荐: 0.01-0.03)',
            'fuzzy_derror_medium': '中等误差变化率阈值 (推荐: 0.08-0.12)'
        }
        
        for param_name, description in params.items():
            value = input(f"输入 {description} [回车跳过]: ")
            if value.strip():
                try:
                    self.set_parameter(param_name, float(value))
                except ValueError:
                    print("无效数值")
    
    def quick_anti_shake_setup(self):
        """快速抗抖动设置"""
        print("\n--- 快速抗抖动设置 ---")
        print("应用一组预设的抗抖动参数...")
        
        # 保守的抗抖动参数
        anti_shake_params = {
            'yaw_kp': 0.04,
            'yaw_ki': 0.0,
            'yaw_kd': 0.008,
            'pitch_kp': 0.04,
            'pitch_ki': 0.0,
            'pitch_kd': 0.008,
            'deadzone_pixel': 4.0,
            'filter_alpha': 0.75,
            'min_movement_threshold': 0.0015,
            'min_confidence': 0.85,
            'max_angle_change': 0.015,
            'fuzzy_error_small': 0.05,
            'fuzzy_error_medium': 0.2,
            'fuzzy_derror_small': 0.02,
            'fuzzy_derror_medium': 0.1
        }
        
        confirm = input("确认应用抗抖动参数? (y/n): ")
        if confirm.lower() == 'y':
            success_count = 0
            for param_name, value in anti_shake_params.items():
                if self.set_parameter(param_name, value):
                    success_count += 1
            
            print(f"\n成功应用 {success_count}/{len(anti_shake_params)} 个参数")
            print("建议观察云台行为并根据需要微调")

def main():
    rclpy.init()
    
    tuner = FuzzyPIDTuner()
    
    try:
        tuner.interactive_tuning()
    except KeyboardInterrupt:
        pass
    finally:
        tuner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
