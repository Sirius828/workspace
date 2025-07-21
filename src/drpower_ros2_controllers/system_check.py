#!/usr/bin/env python3
"""
DrPower ROS2 Controllers 系统状态检查脚本
检查系统配置、依赖和运行状态
"""

import os
import sys
import subprocess
import time
import socket
from pathlib import Path

class DrPowerSystemChecker:
    """DrPower系统检查器"""
    
    def __init__(self):
        self.results = {}
        self.issues = []
        
    def run_command(self, command, capture_output=True):
        """安全地运行系统命令"""
        try:
            result = subprocess.run(
                command, 
                shell=True, 
                capture_output=capture_output, 
                text=True,
                timeout=10
            )
            return result.returncode, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return -1, "", "Command timeout"
        except Exception as e:
            return -1, "", str(e)
    
    def check_ros2_installation(self):
        """检查ROS2安装"""
        print("🔍 检查ROS2安装...")
        
        # 检查ROS2环境变量
        ros_distro = os.environ.get('ROS_DISTRO')
        if not ros_distro:
            self.issues.append("ROS_DISTRO环境变量未设置")
            self.results['ros2_env'] = False
        else:
            print(f"   ✅ ROS2版本: {ros_distro}")
            self.results['ros2_env'] = True
            
        # 检查ros2命令
        ret, out, err = self.run_command("which ros2")
        if ret != 0:
            self.issues.append("ros2命令未找到")
            self.results['ros2_command'] = False
        else:
            print(f"   ✅ ros2命令路径: {out.strip()}")
            self.results['ros2_command'] = True
            
        # 检查核心包
        core_packages = [
            'controller_manager',
            'hardware_interface', 
            'ros2_control',
            'ros2_controllers'
        ]
        
        missing_packages = []
        for package in core_packages:
            ret, _, _ = self.run_command(f"ros2 pkg list | grep {package}")
            if ret != 0:
                missing_packages.append(package)
                
        if missing_packages:
            self.issues.append(f"缺少核心包: {', '.join(missing_packages)}")
            self.results['core_packages'] = False
        else:
            print("   ✅ 核心包完整")
            self.results['core_packages'] = True
    
    def check_drpower_packages(self):
        """检查DrPower包"""
        print("\n🔍 检查DrPower包...")
        
        # 检查DrPower包
        drpower_packages = [
            'drpower_hardware_interface',
            'drpower_moveit_config'
        ]
        
        for package in drpower_packages:
            ret, out, err = self.run_command(f"ros2 pkg list | grep {package}")
            if ret != 0:
                self.issues.append(f"DrPower包未找到: {package}")
                self.results[f'{package}_installed'] = False
            else:
                print(f"   ✅ {package} 已安装")
                self.results[f'{package}_installed'] = True
                
                # 检查包路径
                ret, path, _ = self.run_command(f"ros2 pkg prefix {package}")
                if ret == 0:
                    print(f"      路径: {path.strip()}")
    
    def check_hardware_connections(self):
        """检查硬件连接"""
        print("\n🔍 检查硬件连接...")
        
        # 检查USB设备
        usb_devices = []
        for device in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0']:
            if os.path.exists(device):
                usb_devices.append(device)
                
        if usb_devices:
            print(f"   ✅ 找到USB设备: {', '.join(usb_devices)}")
            self.results['usb_devices'] = True
            
            # 检查设备权限
            for device in usb_devices:
                stat = os.stat(device)
                mode = oct(stat.st_mode)[-3:]
                print(f"      {device}: 权限 {mode}")
                if mode < '666':
                    self.issues.append(f"设备权限不足: {device} (需要666)")
        else:
            self.issues.append("未找到USB串口设备 (/dev/ttyUSB* 或 /dev/ttyACM*)")
            self.results['usb_devices'] = False
            
        # 检查用户组
        ret, groups, _ = self.run_command("groups")
        if 'dialout' not in groups:
            self.issues.append("用户不在dialout组中，可能无法访问串口设备")
            self.results['dialout_group'] = False
        else:
            print("   ✅ 用户在dialout组中")
            self.results['dialout_group'] = True
    
    def check_system_resources(self):
        """检查系统资源"""
        print("\n🔍 检查系统资源...")
        
        # 检查CPU
        ret, cpu_info, _ = self.run_command("nproc")
        if ret == 0:
            cpu_count = int(cpu_info.strip())
            print(f"   ✅ CPU核心数: {cpu_count}")
            if cpu_count < 2:
                self.issues.append("CPU核心数过少，建议至少2核")
                
        # 检查内存
        ret, mem_info, _ = self.run_command("free -m")
        if ret == 0:
            lines = mem_info.strip().split('\n')
            if len(lines) > 1:
                mem_total = int(lines[1].split()[1])
                print(f"   ✅ 总内存: {mem_total} MB")
                if mem_total < 2048:
                    self.issues.append("内存不足，建议至少2GB")
                    
        # 检查磁盘空间
        ret, disk_info, _ = self.run_command("df -h .")
        if ret == 0:
            lines = disk_info.strip().split('\n')
            if len(lines) > 1:
                disk_usage = lines[1].split()[4]
                print(f"   ✅ 磁盘使用率: {disk_usage}")
                if int(disk_usage.rstrip('%')) > 90:
                    self.issues.append("磁盘空间不足")
    
    def check_network_configuration(self):
        """检查网络配置"""
        print("\n🔍 检查网络配置...")
        
        # 检查ROS_DOMAIN_ID
        domain_id = os.environ.get('ROS_DOMAIN_ID')
        if domain_id:
            print(f"   ✅ ROS_DOMAIN_ID: {domain_id}")
        else:
            print("   ⚠️  ROS_DOMAIN_ID未设置 (使用默认值0)")
            
        # 检查localhost通信
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            result = sock.connect_ex(('127.0.0.1', 11511))  # ROS2 DDS端口
            sock.close()
            if result == 0:
                print("   ✅ localhost通信正常")
            else:
                print("   ⚠️  localhost通信可能有问题")
        except Exception:
            print("   ⚠️  无法测试网络连接")
    
    def check_running_processes(self):
        """检查运行的ROS2进程"""
        print("\n🔍 检查运行的ROS2进程...")
        
        # 检查ros2进程
        ret, processes, _ = self.run_command("ps aux | grep ros2")
        if ret == 0:
            ros2_processes = [line for line in processes.split('\n') 
                            if 'ros2' in line and 'grep' not in line]
            if ros2_processes:
                print(f"   ✅ 找到 {len(ros2_processes)} 个ROS2进程")
                for proc in ros2_processes[:3]:  # 显示前3个
                    print(f"      {proc.split()[10:12]}")
            else:
                print("   ℹ️  当前没有运行的ROS2进程")
                
        # 检查DDS进程
        ret, dds_processes, _ = self.run_command("ps aux | grep dds")
        if ret == 0:
            dds_count = len([line for line in dds_processes.split('\n') 
                           if 'dds' in line and 'grep' not in line])
            if dds_count > 0:
                print(f"   ✅ DDS进程运行正常 ({dds_count}个)")
    
    def check_workspace_setup(self):
        """检查工作空间设置"""
        print("\n🔍 检查工作空间设置...")
        
        # 检查当前目录
        current_dir = os.getcwd()
        print(f"   当前目录: {current_dir}")
        
        # 查找工作空间
        workspace_indicators = ['src', 'build', 'install', 'log']
        if all(os.path.exists(ind) for ind in workspace_indicators):
            print("   ✅ 当前目录是ROS2工作空间")
            self.results['workspace_setup'] = True
            
            # 检查DrPower包源码
            drpower_src = Path('src/drpower_ros2_controllers')
            if drpower_src.exists():
                print("   ✅ DrPower源码包存在")
                self.results['drpower_source'] = True
            else:
                self.issues.append("DrPower源码包不存在")
                self.results['drpower_source'] = False
                
            # 检查编译输出
            drpower_install = Path('install/drpower_hardware_interface')
            if drpower_install.exists():
                print("   ✅ DrPower已编译")
                self.results['drpower_built'] = True
            else:
                self.issues.append("DrPower包未编译或编译失败")
                self.results['drpower_built'] = False
        else:
            self.issues.append("当前目录不是ROS2工作空间")
            self.results['workspace_setup'] = False
    
    def suggest_fixes(self):
        """建议修复方案"""
        if not self.issues:
            return
            
        print("\n🔧 修复建议:")
        for i, issue in enumerate(self.issues, 1):
            print(f"{i}. {issue}")
            
            # 提供具体的修复命令
            if "ROS_DISTRO" in issue:
                print("   修复: source /opt/ros/humble/setup.bash")
            elif "ros2命令" in issue:
                print("   修复: sudo apt install ros-humble-desktop")
            elif "缺少核心包" in issue:
                print("   修复: sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers")
            elif "设备权限" in issue:
                print("   修复: sudo chmod 666 /dev/ttyUSB0 或 sudo usermod -a -G dialout $USER")
            elif "dialout组" in issue:
                print("   修复: sudo usermod -a -G dialout $USER && 重新登录")
            elif "未编译" in issue:
                print("   修复: colcon build --packages-select drpower_hardware_interface")
    
    def generate_report(self):
        """生成检查报告"""
        print(f"\n📊 系统检查报告")
        print("=" * 50)
        
        total_checks = len(self.results)
        passed_checks = sum(1 for v in self.results.values() if v)
        
        print(f"总检查项: {total_checks}")
        print(f"通过检查: {passed_checks}")
        print(f"成功率: {passed_checks/total_checks*100:.1f}%")
        
        if self.issues:
            print(f"发现问题: {len(self.issues)}")
        else:
            print("🎉 所有检查项目通过！")
            
        # 保存报告到文件
        report_file = "drpower_system_check_report.txt"
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write(f"DrPower ROS2 Controllers 系统检查报告\n")
            f.write(f"检查时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"{'='*50}\n\n")
            
            f.write("检查结果:\n")
            for key, value in self.results.items():
                status = "✅ 通过" if value else "❌ 失败"
                f.write(f"{key}: {status}\n")
                
            if self.issues:
                f.write(f"\n发现的问题:\n")
                for i, issue in enumerate(self.issues, 1):
                    f.write(f"{i}. {issue}\n")
                    
        print(f"\n📄 详细报告已保存到: {report_file}")
    
    def run_all_checks(self):
        """运行所有检查"""
        print("🚀 DrPower ROS2 Controllers 系统检查")
        print("=" * 50)
        
        try:
            self.check_ros2_installation()
            self.check_drpower_packages()
            self.check_hardware_connections()
            self.check_system_resources()
            self.check_network_configuration()
            self.check_running_processes()
            self.check_workspace_setup()
            
            self.suggest_fixes()
            self.generate_report()
            
        except KeyboardInterrupt:
            print("\n\n⏹️  检查被用户中断")
        except Exception as e:
            print(f"\n\n❌ 检查过程中发生错误: {e}")


def main():
    """主函数"""
    checker = DrPowerSystemChecker()
    checker.run_all_checks()


if __name__ == '__main__':
    main()
