#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import tkinter as tk
from tkinter import ttk, messagebox
import threading

class StartEngineGUI(Node):
    def __init__(self):
        super().__init__('start_engine_gui')
        
        # 服务客户端
        self.start_client = self.create_client(Empty, '/start')
        
        # 订阅里程计
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/chassis/odom',
            self.odom_callback,
            10
        )
        
        # 里程计数据
        self.current_odom = None
        
        # GUI初始化
        self.setup_gui()
        
        self.get_logger().info('Start Engine GUI initialized')
    
    def setup_gui(self):
        """设置GUI界面"""
        self.root = tk.Tk()
        self.root.title("Start Engine Control")
        
        # 获取屏幕尺寸并设置全屏
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        self.root.geometry(f"{screen_width}x{screen_height}+0+0")
        self.root.resizable(True, True)
        
        # 记录全屏状态
        self.is_fullscreen = True
        
        # 设置样式
        style = ttk.Style()
        style.configure('Start.TButton', font=('Arial', 18, 'bold'), padding=(30, 15))
        style.configure('Exit.TButton', font=('Arial', 14, 'bold'), padding=(20, 10))
        
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 创建左侧控制框架
        control_frame = ttk.Frame(main_frame, padding="30")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 创建右侧信息框架
        info_frame = ttk.LabelFrame(main_frame, text="机器人状态", padding="20")
        info_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(20, 0))
        
        # 状态标签（左侧）
        self.status_label = ttk.Label(
            control_frame, 
            text="准备就绪", 
            font=('Arial', 14),
            anchor='center'
        )
        self.status_label.grid(row=0, column=0, pady=(0, 40), columnspan=2)
        
        # 启动按钮（左侧）
        self.start_button = ttk.Button(
            control_frame,
            text="启动系统",
            style='Start.TButton',
            command=self.on_start_clicked
        )
        self.start_button.grid(row=1, column=0, pady=20, padx=20, ipadx=40, ipady=20, columnspan=2)
        
        # 退出按钮（左侧）
        self.exit_button = ttk.Button(
            control_frame,
            text="退出程序",
            style='Exit.TButton',
            command=self.on_exit_clicked
        )
        self.exit_button.grid(row=3, column=0, pady=20, padx=20, ipadx=30, ipady=15, columnspan=2)
        
        # 进度条（左侧，隐藏状态）
        self.progress = ttk.Progressbar(
            control_frame,
            mode='indeterminate',
            length=400
        )
        
        # 右侧里程计信息
        ttk.Label(info_frame, text="位置信息", font=('Arial', 12, 'bold')).grid(row=0, column=0, columnspan=2, pady=(0, 10))
        
        ttk.Label(info_frame, text="X:", font=('Arial', 10)).grid(row=1, column=0, sticky='w', pady=2)
        self.x_label = ttk.Label(info_frame, text="--", font=('Arial', 10))
        self.x_label.grid(row=1, column=1, sticky='w', pady=2, padx=(10, 0))
        
        ttk.Label(info_frame, text="Y:", font=('Arial', 10)).grid(row=2, column=0, sticky='w', pady=2)
        self.y_label = ttk.Label(info_frame, text="--", font=('Arial', 10))
        self.y_label.grid(row=2, column=1, sticky='w', pady=2, padx=(10, 0))
        
        ttk.Label(info_frame, text="Z:", font=('Arial', 10)).grid(row=3, column=0, sticky='w', pady=2)
        self.z_label = ttk.Label(info_frame, text="--", font=('Arial', 10))
        self.z_label.grid(row=3, column=1, sticky='w', pady=2, padx=(10, 0))
        
        ttk.Label(info_frame, text="姿态信息", font=('Arial', 12, 'bold')).grid(row=4, column=0, columnspan=2, pady=(20, 10))
        
        ttk.Label(info_frame, text="Yaw:", font=('Arial', 10)).grid(row=5, column=0, sticky='w', pady=2)
        self.yaw_label = ttk.Label(info_frame, text="--", font=('Arial', 10))
        self.yaw_label.grid(row=5, column=1, sticky='w', pady=2, padx=(10, 0))
        
        ttk.Label(info_frame, text="速度信息", font=('Arial', 12, 'bold')).grid(row=6, column=0, columnspan=2, pady=(20, 10))
        
        ttk.Label(info_frame, text="线速度:", font=('Arial', 10)).grid(row=7, column=0, sticky='w', pady=2)
        self.linear_vel_label = ttk.Label(info_frame, text="--", font=('Arial', 10))
        self.linear_vel_label.grid(row=7, column=1, sticky='w', pady=2, padx=(10, 0))
        
        ttk.Label(info_frame, text="角速度:", font=('Arial', 10)).grid(row=8, column=0, sticky='w', pady=2)
        self.angular_vel_label = ttk.Label(info_frame, text="--", font=('Arial', 10))
        self.angular_vel_label.grid(row=8, column=1, sticky='w', pady=2, padx=(10, 0))
        
        # 配置网格权重
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        main_frame.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=1)
        control_frame.grid_rowconfigure(0, weight=1)
        control_frame.grid_rowconfigure(1, weight=1)
        control_frame.grid_rowconfigure(2, weight=0)
        control_frame.grid_rowconfigure(3, weight=1)
        control_frame.grid_columnconfigure(0, weight=1)
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 绑定ESC键退出全屏
        self.root.bind('<Escape>', self.toggle_fullscreen)
        self.root.bind('<F11>', self.toggle_fullscreen)
    
    def on_start_clicked(self):
        """启动按钮点击事件"""
        self.get_logger().info('Start button clicked')
        
        # 更新UI状态
        self.start_button.config(state='disabled')
        self.exit_button.config(state='disabled')
        self.status_label.config(text="正在启动...")
        self.progress.grid(row=2, column=0, pady=20, columnspan=2)
        self.progress.start()
        
        # 在后台线程中调用服务
        threading.Thread(target=self.call_start_service, daemon=True).start()
    
    def odom_callback(self, msg):
        """里程计数据回调"""
        self.current_odom = msg
        
        # 更新GUI显示
        try:
            # 位置信息
            x = f"{msg.pose.pose.position.x:.3f} m"
            y = f"{msg.pose.pose.position.y:.3f} m" 
            z = f"{msg.pose.pose.position.z:.3f} m"
            
            # 计算yaw角度
            import math
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            # 四元数转欧拉角 (只计算yaw)
            yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
            yaw_deg = f"{math.degrees(yaw):.1f}°"
            
            # 速度信息
            linear_vel = f"{msg.twist.twist.linear.x:.3f} m/s"
            angular_vel = f"{msg.twist.twist.angular.z:.3f} rad/s"
            
            # 在主线程中更新GUI
            self.root.after(0, self.update_odom_display, x, y, z, yaw_deg, linear_vel, angular_vel)
            
        except Exception as e:
            self.get_logger().warn(f'Error updating odom display: {str(e)}')
    
    def update_odom_display(self, x, y, z, yaw, linear_vel, angular_vel):
        """更新里程计显示"""
        try:
            self.x_label.config(text=x)
            self.y_label.config(text=y)
            self.z_label.config(text=z)
            self.yaw_label.config(text=yaw)
            self.linear_vel_label.config(text=linear_vel)
            self.angular_vel_label.config(text=angular_vel)
        except:
            pass
    
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
        self.exit_button.config(state='normal')
        
        # 显示成功消息
        messagebox.showinfo("成功", "系统启动成功！")
    
    def on_service_error(self):
        """服务调用失败回调"""
        self.progress.stop()
        self.progress.grid_remove()
        self.status_label.config(text="启动失败")
        self.start_button.config(state='normal')
        self.exit_button.config(state='normal')
        
        # 显示错误消息
        messagebox.showerror("错误", "服务调用失败，请检查服务是否可用")
    
    def on_service_unavailable(self):
        """服务不可用回调"""
        self.progress.stop()
        self.progress.grid_remove()
        self.status_label.config(text="服务不可用")
        self.start_button.config(state='normal')
        self.exit_button.config(state='normal')
        
        # 显示警告消息
        messagebox.showwarning("警告", "启动服务当前不可用，请稍后重试")
    
    def on_exit_clicked(self):
        """退出按钮点击事件"""
        self.get_logger().info('Exit button clicked')
        self.on_closing()
    
    def toggle_fullscreen(self, event=None):
        """切换全屏模式"""
        if self.is_fullscreen:
            # 切换到窗口模式
            self.root.geometry("300x150")
            self.is_fullscreen = False
        else:
            # 切换到全屏模式
            screen_width = self.root.winfo_screenwidth()
            screen_height = self.root.winfo_screenheight()
            self.root.geometry(f"{screen_width}x{screen_height}+0+0")
            self.is_fullscreen = True
    
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
