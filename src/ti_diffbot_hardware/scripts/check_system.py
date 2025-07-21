#!/usr/bin/env python3
"""
系统状态检查器 - 验证ROS2节点和话题状态
"""

import subprocess
import time
import sys

def run_command(cmd):
    """运行命令并返回结果"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "命令超时"

def check_nodes():
    """检查ROS2节点状态"""
    print("🔍 检查ROS2节点...")
    
    success, output, error = run_command("ros2 node list")
    if not success:
        print(f"❌ 无法获取节点列表: {error}")
        return False
        
    nodes = output.strip().split('\n') if output.strip() else []
    print(f"📊 发现 {len(nodes)} 个节点:")
    
    for node in nodes:
        if node.strip():
            print(f"  ✅ {node.strip()}")
    
    # 检查关键节点
    required_nodes = [
        "/controller_manager",
        "/robot_state_publisher"
    ]
    
    missing_nodes = []
    for req_node in required_nodes:
        if req_node not in output:
            missing_nodes.append(req_node)
    
    if missing_nodes:
        print(f"⚠️  缺少关键节点: {missing_nodes}")
        return False
    else:
        print("✅ 所有关键节点都在运行")
        return True

def check_controllers():
    """检查控制器状态"""
    print("\n🎮 检查控制器状态...")
    
    success, output, error = run_command("ros2 control list_controllers")
    if not success:
        print(f"❌ 无法获取控制器列表: {error}")
        return False
    
    print("📊 控制器状态:")
    print(output)
    
    # 检查diff_drive_controller是否active
    if "diff_drive_controller" in output and "active" in output:
        print("✅ diff_drive_controller 状态正常")
        return True
    else:
        print("❌ diff_drive_controller 未激活")
        return False

def check_topics():
    """检查关键话题"""
    print("\n📡 检查关键话题...")
    
    important_topics = [
        "/diff_drive_controller/odom",
        "/joint_states", 
        "/diff_drive_controller/cmd_vel_unstamped"
    ]
    
    all_good = True
    
    for topic in important_topics:
        success, output, error = run_command(f"ros2 topic info {topic}")
        if success:
            print(f"  ✅ {topic}")
        else:
            print(f"  ❌ {topic} - 不存在")
            all_good = False
    
    return all_good

def check_data_flow():
    """检查数据流"""
    print("\n🌊 检查数据流...")
    
    # 检查里程计数据
    print("  检查里程计数据...")
    success, output, error = run_command("timeout 3 ros2 topic echo /diff_drive_controller/odom --once")
    if success and output.strip():
        print("  ✅ 里程计数据正常")
    else:
        print("  ❌ 里程计数据异常")
        return False
    
    # 检查关节状态数据
    print("  检查关节状态数据...")
    success, output, error = run_command("timeout 3 ros2 topic echo /joint_states --once")
    if success and output.strip():
        print("  ✅ 关节状态数据正常")
    else:
        print("  ❌ 关节状态数据异常")
        return False
    
    return True

def main():
    print("🤖 ROS2 系统状态检查器")
    print("=" * 50)
    
    checks = [
        ("节点检查", check_nodes),
        ("控制器检查", check_controllers), 
        ("话题检查", check_topics),
        ("数据流检查", check_data_flow)
    ]
    
    results = []
    
    for check_name, check_func in checks:
        print(f"\n{check_name}...")
        try:
            result = check_func()
            results.append((check_name, result))
        except Exception as e:
            print(f"❌ {check_name}发生错误: {e}")
            results.append((check_name, False))
    
    # 总结
    print("\n" + "=" * 50)
    print("📋 检查结果总结:")
    print("=" * 50)
    
    all_passed = True
    for check_name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{check_name}: {status}")
        if not result:
            all_passed = False
    
    print("\n" + "=" * 50)
    if all_passed:
        print("🎉 所有检查都通过！系统状态良好。")
        print("💡 现在可以运行底盘测试:")
        print("   ros2 run ti_diffbot_hardware test_chassis.py")
    else:
        print("⚠️  某些检查失败，请解决问题后重试。")
        print("💡 建议:")
        print("   1. 确保启动了完整系统: ros2 launch ti_diffbot_hardware gimbal_complete_system.launch.py")
        print("   2. 检查硬件连接和串口通信")
        print("   3. 查看相关节点的日志信息")
    
    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())
