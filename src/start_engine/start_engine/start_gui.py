#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import tkinter as tk
from tkinter import ttk, messagebox
import threading

class StartEngineGUI(Node):
    def __init__(self):
        super().__init__('start_engine_gui')
        
        # 服务客户端
        self.start_client = self.create_client(Empty, '/start')
        
        # GUI初始化
        self.setup_gui()
        
        self.get_logger().info('Start Engine GUI initialized')
    
    def setup_gui(self):
        """设置GUI界面"""
        self.root = tk.Tk()
        self.root.title("Start Engine Control")
        self.root.geometry("300x150")
        self.root.resizable(False, False)
        
        # 设置样式
        style = ttk.Style()
        style.configure('Start.TButton', font=('Arial', 12, 'bold'))
        
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 状态标签
        self.status_label = ttk.Label(
            main_frame, 
            text="准备就绪", 
            font=('Arial', 10)
        )
        self.status_label.grid(row=0, column=0, pady=(0, 20))
        
        # 启动按钮
        self.start_button = ttk.Button(
            main_frame,
            text="启动系统",
            style='Start.TButton',
            command=self.on_start_clicked
        )
        self.start_button.grid(row=1, column=0, pady=10, ipadx=20, ipady=10)
        
        # 进度条（隐藏状态）
        self.progress = ttk.Progressbar(
            main_frame,
            mode='indeterminate',
            length=200
        )
        
        # 配置网格权重
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        main_frame.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def on_start_clicked(self):
        """启动按钮点击事件"""
        self.get_logger().info('Start button clicked')
        
        # 更新UI状态
        self.start_button.config(state='disabled')
        self.status_label.config(text="正在启动...")
        self.progress.grid(row=2, column=0, pady=10)
        self.progress.start()
        
        # 在后台线程中调用服务
        threading.Thread(target=self.call_start_service, daemon=True).start()
    
    def call_start_service(self):
        """调用启动服务"""
        try:
            # 等待服务可用
            if not self.start_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn('Start service not available')
                self.root.after(0, self.on_service_unavailable)
                return
            
            # 创建请求
            request = Empty.Request()
            
            # 调用服务
            future = self.start_client.call_async(request)
            
            # 等待响应
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                self.get_logger().info('Start service called successfully')
                self.root.after(0, self.on_service_success)
            else:
                self.get_logger().error('Failed to call start service')
                self.root.after(0, self.on_service_error)
                
        except Exception as e:
            self.get_logger().error(f'Exception in service call: {str(e)}')
            self.root.after(0, self.on_service_error)
    
    def on_service_success(self):
        """服务调用成功回调"""
        self.progress.stop()
        self.progress.grid_remove()
        self.status_label.config(text="启动成功！")
        self.start_button.config(state='normal')
        
        # 显示成功消息
        messagebox.showinfo("成功", "系统启动成功！")
    
    def on_service_error(self):
        """服务调用失败回调"""
        self.progress.stop()
        self.progress.grid_remove()
        self.status_label.config(text="启动失败")
        self.start_button.config(state='normal')
        
        # 显示错误消息
        messagebox.showerror("错误", "服务调用失败，请检查服务是否可用")
    
    def on_service_unavailable(self):
        """服务不可用回调"""
        self.progress.stop()
        self.progress.grid_remove()
        self.status_label.config(text="服务不可用")
        self.start_button.config(state='normal')
        
        # 显示警告消息
        messagebox.showwarning("警告", "启动服务当前不可用，请稍后重试")
    
    def on_closing(self):
        """窗口关闭事件"""
        self.get_logger().info('GUI closing')
        try:
            self.root.quit()
        except:
            pass
        try:
            self.root.destroy()
        except:
            pass
    
    def run(self):
        """运行GUI"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt received')
        except Exception as e:
            self.get_logger().error(f'GUI error: {str(e)}')
        finally:
            try:
                self.on_closing()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    
    gui_node = StartEngineGUI()
    
    # 在单独线程中运行ROS2
    ros_thread = threading.Thread(target=rclpy.spin, args=(gui_node,), daemon=True)
    ros_thread.start()
    
    try:
        # 运行GUI主循环
        gui_node.run()
    except KeyboardInterrupt:
        gui_node.get_logger().info('Shutting down GUI')
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
