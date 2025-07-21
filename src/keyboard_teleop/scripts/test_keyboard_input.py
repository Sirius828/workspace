#!/usr/bin/env python3
"""
键盘输入测试脚本
用于诊断键盘输入问题
"""

import sys
import termios
import tty
import select
import os


def test_keyboard_input():
    """测试键盘输入方法"""
    print("键盘输入测试")
    print("=" * 30)
    
    # 方法1：标准输入测试
    print("方法1: 基本stdin测试")
    print("请输入一个字符然后按回车: ", end="", flush=True)
    try:
        char = input()
        print(f"收到: '{char}'")
    except Exception as e:
        print(f"错误: {e}")
    
    print()
    
    # 方法2：原始模式测试
    print("方法2: 原始模式测试")
    print("按任意键(Ctrl+C退出):")
    
    if os.isatty(sys.stdin.fileno()):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            count = 0
            while count < 10:  # 最多测试10个字符
                char = sys.stdin.read(1)
                if ord(char) == 3:  # Ctrl+C
                    break
                print(f"\r收到字符: '{char}' (ASCII: {ord(char)}) [{count+1}/10]", end="", flush=True)
                count += 1
        except Exception as e:
            print(f"\n错误: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\n恢复终端设置")
    else:
        print("不是TTY环境，跳过原始模式测试")
    
    print()
    
    # 方法3：非阻塞输入测试
    print("方法3: 非阻塞输入测试")
    print("快速按一些键(5秒测试):")
    
    if os.isatty(sys.stdin.fileno()):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            import time
            start_time = time.time()
            char_count = 0
            
            while time.time() - start_time < 5.0:
                if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                    char = sys.stdin.read(1)
                    char_count += 1
                    print(f"\r收到: '{char}' 总计: {char_count}", end="", flush=True)
                    if ord(char) == 3:  # Ctrl+C
                        break
                
        except Exception as e:
            print(f"\n错误: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\n测试完成")
    else:
        print("不是TTY环境，跳过非阻塞测试")
    
    print("\n测试结束")


if __name__ == '__main__':
    test_keyboard_input()
