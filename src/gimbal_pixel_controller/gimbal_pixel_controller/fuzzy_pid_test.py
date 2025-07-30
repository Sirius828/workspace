#!/usr/bin/env python3
"""
模糊PID控制器测试脚本
用于验证模糊PID控制器的响应特性
"""

import numpy as np
import matplotlib.pyplot as plt
from pixel_controller_node import FuzzyPIDController
import time

def test_fuzzy_pid_response():
    """测试模糊PID控制器对不同误差输入的响应"""
    
    # 创建模糊PID控制器
    fuzzy_pid = FuzzyPIDController(
        base_kp=0.1,
        base_ki=0.01,
        base_kd=0.02,
        output_limit=0.1
    )
    
    # 测试不同的误差值
    test_errors = [-0.5, -0.2, -0.1, -0.05, 0.0, 0.05, 0.1, 0.2, 0.5]
    
    print("模糊PID控制器响应测试")
    print("=" * 60)
    print(f"{'误差':<8} {'输出':<8} {'Kp调整':<8} {'Ki调整':<8} {'Kd调整':<8}")
    print("-" * 60)
    
    results = []
    current_time = time.time()
    
    for error in test_errors:
        output, kp_adj, ki_adj, kd_adj = fuzzy_pid.update(error, current_time)
        print(f"{error:<8.3f} {output:<8.3f} {kp_adj:<8.3f} {ki_adj:<8.3f} {kd_adj:<8.3f}")
        
        results.append({
            'error': error,
            'output': output,
            'kp_adj': kp_adj,
            'ki_adj': ki_adj,
            'kd_adj': kd_adj
        })
        
        current_time += 0.1  # 模拟时间间隔
        fuzzy_pid.reset()    # 重置控制器状态
    
    return results

def test_step_response():
    """测试模糊PID控制器的阶跃响应"""
    
    # 创建模糊PID控制器
    fuzzy_pid = FuzzyPIDController(
        base_kp=0.1,
        base_ki=0.01,
        base_kd=0.02,
        output_limit=0.1
    )
    
    # 传统PID控制器对比
    from pixel_controller_node import PIDController
    traditional_pid = PIDController(
        kp=0.1,
        ki=0.01,
        kd=0.02,
        output_limit=0.1
    )
    
    # 仿真参数
    dt = 0.01  # 时间步长
    total_time = 5.0  # 总仿真时间
    setpoint = 0.0  # 目标值
    initial_error = 0.3  # 初始误差
    
    time_points = np.arange(0, total_time, dt)
    fuzzy_outputs = []
    traditional_outputs = []
    errors = []
    
    # 仿真系统（简单的一阶系统）
    plant_output_fuzzy = 0.0
    plant_output_traditional = 0.0
    
    print(f"\n阶跃响应测试 (初始误差: {initial_error})")
    print("=" * 40)
    
    for t in time_points:
        # 计算误差
        error_fuzzy = setpoint - plant_output_fuzzy
        error_traditional = setpoint - plant_output_traditional
        
        # 控制器输出
        fuzzy_control, _, _, _ = fuzzy_pid.update(error_fuzzy, t)
        traditional_control = traditional_pid.update(error_traditional, t)
        
        # 简单的植物模型 (一阶惯性环节)
        tau = 1.0  # 时间常数
        plant_output_fuzzy += dt * (fuzzy_control - plant_output_fuzzy) / tau
        plant_output_traditional += dt * (traditional_control - plant_output_traditional) / tau
        
        # 记录数据
        fuzzy_outputs.append(plant_output_fuzzy)
        traditional_outputs.append(plant_output_traditional)
        errors.append(error_fuzzy)
    
    # 设置初始值
    fuzzy_outputs[0] = initial_error
    traditional_outputs[0] = initial_error
    
    return time_points, fuzzy_outputs, traditional_outputs, errors

def plot_comparison(time_points, fuzzy_outputs, traditional_outputs):
    """绘制对比图"""
    try:
        plt.figure(figsize=(12, 8))
        
        # 阶跃响应对比
        plt.subplot(2, 1, 1)
        plt.plot(time_points, fuzzy_outputs, 'b-', label='模糊PID', linewidth=2)
        plt.plot(time_points, traditional_outputs, 'r--', label='传统PID', linewidth=2)
        plt.axhline(y=0, color='k', linestyle=':', alpha=0.5, label='目标值')
        plt.xlabel('时间 (秒)')
        plt.ylabel('系统输出')
        plt.title('模糊PID vs 传统PID 阶跃响应对比')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # 误差对比
        plt.subplot(2, 1, 2)
        fuzzy_errors = [abs(0 - output) for output in fuzzy_outputs]
        traditional_errors = [abs(0 - output) for output in traditional_outputs]
        
        plt.plot(time_points, fuzzy_errors, 'b-', label='模糊PID误差', linewidth=2)
        plt.plot(time_points, traditional_errors, 'r--', label='传统PID误差', linewidth=2)
        plt.xlabel('时间 (秒)')
        plt.ylabel('绝对误差')
        plt.title('控制误差对比')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('/tmp/fuzzy_pid_comparison.png', dpi=300, bbox_inches='tight')
        print(f"\n对比图已保存到: /tmp/fuzzy_pid_comparison.png")
        
    except ImportError:
        print("matplotlib未安装，跳过绘图")

def main():
    """主测试函数"""
    print("模糊PID控制器测试开始")
    print("=" * 50)
    
    # 测试1: 响应特性测试
    print("\n1. 模糊PID响应特性测试")
    test_fuzzy_pid_response()
    
    # 测试2: 阶跃响应测试
    print("\n2. 阶跃响应对比测试")
    time_points, fuzzy_outputs, traditional_outputs, errors = test_step_response()
    
    # 性能指标计算
    settling_time_fuzzy = None
    settling_time_traditional = None
    
    for i, (f_out, t_out) in enumerate(zip(fuzzy_outputs, traditional_outputs)):
        if settling_time_fuzzy is None and abs(f_out) < 0.02:  # 2%误差带
            settling_time_fuzzy = time_points[i]
        if settling_time_traditional is None and abs(t_out) < 0.02:
            settling_time_traditional = time_points[i]
        
        if settling_time_fuzzy and settling_time_traditional:
            break
    
    print(f"模糊PID稳定时间: {settling_time_fuzzy:.2f}秒")
    print(f"传统PID稳定时间: {settling_time_traditional:.2f}秒")
    
    # 绘制对比图
    plot_comparison(time_points, fuzzy_outputs, traditional_outputs)
    
    print("\n测试完成!")

if __name__ == '__main__':
    main()
