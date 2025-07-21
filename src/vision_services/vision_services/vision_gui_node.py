#!/usr/bin/env python3
"""
Vision Services GUI Node
提供图形界面来调用四个视觉服务
"""

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox
import threading
from vision_services_msgs.srv import Reset, SquareLoop, A4Loop, A4LoopRot


class VisionGUINode(Node):
    """视觉服务GUI节点"""
    
    def __init__(self):
        super().__init__('vision_gui_node')
        
        # 创建服务客户端
        self.reset_client = self.create_client(Reset, 'vision_services/reset')
        self.square_loop_client = self.create_client(SquareLoop, 'vision_services/square_loop')
        self.a4_loop_client = self.create_client(A4Loop, 'vision_services/a4_loop')
        self.a4_loop_rot_client = self.create_client(A4LoopRot, 'vision_services/a4_loop_rot')
        
        # 等待服务可用
        self.get_logger().info('等待视觉服务可用...')
        self._wait_for_services()
        
        # 初始化GUI
        self._init_gui()
        
        self.get_logger().info('Vision GUI Node initialized')
    
    def _wait_for_services(self):
        """等待所有服务可用"""
        services = [
            (self.reset_client, 'reset'),
            (self.square_loop_client, 'square_loop'),
            (self.a4_loop_client, 'a4_loop'),
            (self.a4_loop_rot_client, 'a4_loop_rot')
        ]
        
        for client, name in services:
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().warn(f'Service {name} not available within timeout')
            else:
                self.get_logger().info(f'Service {name} is available')
    
    def _init_gui(self):
        """初始化GUI界面"""
        self.root = tk.Tk()
        self.root.title('Vision Services Controller')
        
        # 设置全屏
        self.root.attributes('-fullscreen', True)
        self.root.configure(bg='#2c3e50')
        
        # 按ESC键退出全屏
        self.root.bind('<Escape>', lambda e: self.root.quit())
        
        # 创建主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill='both', expand=True, padx=20, pady=20)
        
        # 配置样式
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Big.TButton', 
                       font=('Arial', 24, 'bold'),
                       padding=20)
        
        # 创建标题
        title_label = ttk.Label(main_frame, 
                               text='Vision Services Controller', 
                               font=('Arial', 32, 'bold'))
        title_label.pack(pady=(0, 30))
        
        # 创建按钮框架 (2x2 网格)
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill='both', expand=True)
        
        # 配置网格权重
        button_frame.grid_columnconfigure(0, weight=1)
        button_frame.grid_columnconfigure(1, weight=1)
        button_frame.grid_rowconfigure(0, weight=1)
        button_frame.grid_rowconfigure(1, weight=1)
        
        # 创建四个按钮
        self.create_service_button(button_frame, 'RESET', self._call_reset, 0, 0, '#e74c3c')
        self.create_service_button(button_frame, 'SQUARE_LOOP', self._call_square_loop, 0, 1, '#3498db')
        self.create_service_button(button_frame, 'A4_LOOP', self._call_a4_loop, 1, 0, '#2ecc71')
        self.create_service_button(button_frame, 'A4_LOOP_ROT', self._call_a4_loop_rot, 1, 1, '#f39c12')
        
        # 创建状态栏
        self.status_var = tk.StringVar()
        self.status_var.set('Ready')
        status_label = ttk.Label(main_frame, 
                                textvariable=self.status_var,
                                font=('Arial', 14),
                                foreground='#34495e')
        status_label.pack(pady=(20, 0))
        
        # 创建退出按钮
        exit_button = ttk.Button(main_frame, 
                                text='Exit (ESC)',
                                command=self.root.quit,
                                style='Big.TButton')
        exit_button.pack(pady=(20, 0))
    
    def create_service_button(self, parent, text, command, row, col, color):
        """创建服务按钮"""
        # 创建按钮框架
        button_frame = tk.Frame(parent, bg=color, relief='raised', bd=2)
        button_frame.grid(row=row, column=col, padx=10, pady=10, sticky='nsew')
        
        # 创建按钮
        button = tk.Button(button_frame,
                          text=text,
                          command=command,
                          font=('Arial', 20, 'bold'),
                          bg=color,
                          fg='white',
                          relief='flat',
                          bd=0,
                          activebackground=self._darken_color(color),
                          activeforeground='white')
        button.pack(fill='both', expand=True, padx=5, pady=5)
        
        # 添加悬停效果
        button.bind('<Enter>', lambda e: button.config(bg=self._darken_color(color)))
        button.bind('<Leave>', lambda e: button.config(bg=color))
    
    def _darken_color(self, color):
        """使颜色变暗"""
        color_map = {
            '#e74c3c': '#c0392b',
            '#3498db': '#2980b9',
            '#2ecc71': '#27ae60',
            '#f39c12': '#e67e22'
        }
        return color_map.get(color, color)
    
    def _update_status(self, message):
        """更新状态栏"""
        self.status_var.set(message)
        self.root.update_idletasks()
    
    def _call_reset(self):
        """调用RESET服务"""
        self._update_status('Calling RESET service...')
        threading.Thread(target=self._async_call_reset, daemon=True).start()
    
    def _call_square_loop(self):
        """调用SQUARE_LOOP服务"""
        self._update_status('Calling SQUARE_LOOP service...')
        threading.Thread(target=self._async_call_square_loop, daemon=True).start()
    
    def _call_a4_loop(self):
        """调用A4_LOOP服务"""
        self._update_status('Calling A4_LOOP service...')
        threading.Thread(target=self._async_call_a4_loop, daemon=True).start()
    
    def _call_a4_loop_rot(self):
        """调用A4_LOOP_ROT服务"""
        self._update_status('Calling A4_LOOP_ROT service...')
        threading.Thread(target=self._async_call_a4_loop_rot, daemon=True).start()
    
    def _async_call_reset(self):
        """异步调用RESET服务"""
        try:
            request = Reset.Request()
            future = self.reset_client.call_async(request)
            
            # 等待响应
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self._update_status(f'RESET: {response.message}')
                    self.get_logger().info(f'RESET success: {response.message}')
                else:
                    self._update_status(f'RESET failed: {response.message}')
                    self.get_logger().error(f'RESET failed: {response.message}')
            else:
                self._update_status('RESET service call failed')
                self.get_logger().error('RESET service call failed')
        except Exception as e:
            self._update_status(f'RESET error: {str(e)}')
            self.get_logger().error(f'RESET error: {e}')
    
    def _async_call_square_loop(self):
        """异步调用SQUARE_LOOP服务"""
        try:
            request = SquareLoop.Request()
            future = self.square_loop_client.call_async(request)
            
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self._update_status(f'SQUARE_LOOP: {response.message}')
                    self.get_logger().info(f'SQUARE_LOOP success: {response.message}')
                else:
                    self._update_status(f'SQUARE_LOOP failed: {response.message}')
                    self.get_logger().error(f'SQUARE_LOOP failed: {response.message}')
            else:
                self._update_status('SQUARE_LOOP service call failed')
                self.get_logger().error('SQUARE_LOOP service call failed')
        except Exception as e:
            self._update_status(f'SQUARE_LOOP error: {str(e)}')
            self.get_logger().error(f'SQUARE_LOOP error: {e}')
    
    def _async_call_a4_loop(self):
        """异步调用A4_LOOP服务"""
        try:
            request = A4Loop.Request()
            future = self.a4_loop_client.call_async(request)
            
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)  # 更长超时时间
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self._update_status(f'A4_LOOP: {response.message}')
                    self.get_logger().info(f'A4_LOOP success: {response.message}')
                else:
                    self._update_status(f'A4_LOOP failed: {response.message}')
                    self.get_logger().error(f'A4_LOOP failed: {response.message}')
            else:
                self._update_status('A4_LOOP service call failed')
                self.get_logger().error('A4_LOOP service call failed')
        except Exception as e:
            self._update_status(f'A4_LOOP error: {str(e)}')
            self.get_logger().error(f'A4_LOOP error: {e}')
    
    def _async_call_a4_loop_rot(self):
        """异步调用A4_LOOP_ROT服务"""
        try:
            request = A4LoopRot.Request()
            future = self.a4_loop_rot_client.call_async(request)
            
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)  # 更长超时时间
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    msg = f'A4_LOOP_ROT: {response.message}'
                    if hasattr(response, 'rotation_angle'):
                        msg += f' (angle: {response.rotation_angle:.1f}°)'
                    self._update_status(msg)
                    self.get_logger().info(f'A4_LOOP_ROT success: {response.message}')
                else:
                    self._update_status(f'A4_LOOP_ROT failed: {response.message}')
                    self.get_logger().error(f'A4_LOOP_ROT failed: {response.message}')
            else:
                self._update_status('A4_LOOP_ROT service call failed')
                self.get_logger().error('A4_LOOP_ROT service call failed')
        except Exception as e:
            self._update_status(f'A4_LOOP_ROT error: {str(e)}')
            self.get_logger().error(f'A4_LOOP_ROT error: {e}')
    
    def run(self):
        """运行GUI"""
        # 在单独线程中运行ROS2 spin
        def ros_spin():
            rclpy.spin(self)
        
        ros_thread = threading.Thread(target=ros_spin, daemon=True)
        ros_thread.start()
        
        # 运行GUI主循环
        self.root.mainloop()
        
        # 关闭节点
        self.destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = VisionGUINode()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in main: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
