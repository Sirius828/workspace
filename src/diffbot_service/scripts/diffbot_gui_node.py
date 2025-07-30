#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, font
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Trigger  # 修改：使用Trigger而不是Empty
import threading
import queue


class DiffBotGUI(Node):
    def __init__(self):
        super().__init__('diffbot_gui_node')
        
        # 创建服务客户端
        self.start_client = self.create_client(Trigger, '/start')
        
        # 创建订阅者
        self.numbers_subscription = self.create_subscription(
            Int32,
            '/number',  # 修改：订阅/number话题
            self.numbers_callback,
            10
        )
        
        # 存储最新的数字
        self.latest_number = None
        
        # 创建线程安全的队列用于GUI更新
        self.gui_queue = queue.Queue()
        
        # 等待服务可用
        self.get_logger().info('等待 /start 服务...')
        
        # 创建GUI
        self.create_gui()
        
    def numbers_callback(self, msg):
        """处理/number话题的回调"""
        self.latest_number = msg.data
        # 将更新任务放入队列
        self.gui_queue.put(('update_number', msg.data))
        self.get_logger().debug(f'收到数字: {msg.data}')
    
    def call_start_service(self):
        """调用/start服务"""
        if not self.start_client.service_is_ready():
            self.get_logger().warn('/start 服务不可用')
            self.gui_queue.put(('show_message', '服务不可用!'))
            return
        
        # 创建请求
        request = Trigger.Request()
        
        # 异步调用服务
        future = self.start_client.call_async(request)
        future.add_done_callback(self.service_callback)
        
        self.get_logger().info('调用 /start 服务')
        self.gui_queue.put(('show_message', '服务已调用...'))
    
    def service_callback(self, future):
        """服务调用完成的回调"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'/start 服务调用成功: {response.message}')
                self.gui_queue.put(('show_message', f'服务调用成功: {response.message}'))
            else:
                self.get_logger().warn(f'/start 服务调用失败: {response.message}')
                self.gui_queue.put(('show_message', f'服务失败: {response.message}'))
        except Exception as e:
            self.get_logger().error(f'/start 服务调用失败: {e}')
            self.gui_queue.put(('show_message', f'服务调用失败: {e}'))
    
    def create_gui(self):
        """创建GUI界面"""
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title('DiffBot 控制界面')
        self.root.attributes('-fullscreen', True)  # 全屏
        self.root.configure(bg='#2E3440')  # 深色背景
        
        # 获取屏幕尺寸
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        
        # 计算布局尺寸
        left_width = screen_width // 3  # 1/3
        right_width = screen_width - left_width  # 2/3
        
        # 创建左侧框架 (1/3)
        self.left_frame = tk.Frame(
            self.root, 
            width=left_width, 
            height=screen_height,
            bg='#3B4252',
            relief='raised',
            bd=2
        )
        self.left_frame.pack(side='left', fill='both', expand=False)
        self.left_frame.pack_propagate(False)  # 防止框架缩小
        
        # 创建右侧框架 (2/3)
        self.right_frame = tk.Frame(
            self.root, 
            width=right_width, 
            height=screen_height,
            bg='#434C5E',
            relief='raised',
            bd=2
        )
        self.right_frame.pack(side='right', fill='both', expand=True)
        
        # 在左侧添加标题
        title_font = font.Font(family="Arial", size=24, weight="bold")
        title_label = tk.Label(
            self.left_frame,
            text="DiffBot\n控制面板",
            font=title_font,
            bg='#3B4252',
            fg='#ECEFF4',
            pady=30
        )
        title_label.pack(pady=50)
        
        # 在左侧添加启动按钮
        button_font = font.Font(family="Arial", size=18, weight="bold")
        self.start_button = tk.Button(
            self.left_frame,
            text="启动服务\n(/start)",
            font=button_font,
            bg='#5E81AC',
            fg='white',
            activebackground='#81A1C1',
            activeforeground='white',
            relief='raised',
            bd=3,
            padx=30,
            pady=20,
            command=self.on_start_button_click
        )
        self.start_button.pack(pady=30)
        
        # 添加状态标签
        status_font = font.Font(family="Arial", size=12)
        self.status_label = tk.Label(
            self.left_frame,
            text="就绪",
            font=status_font,
            bg='#3B4252',
            fg='#A3BE8C',
            pady=10
        )
        self.status_label.pack(pady=20)
        
        # 添加退出按钮
        exit_font = font.Font(family="Arial", size=14)
        self.exit_button = tk.Button(
            self.left_frame,
            text="退出 (ESC)",
            font=exit_font,
            bg='#BF616A',
            fg='white',
            activebackground='#D08770',
            activeforeground='white',
            relief='raised',
            bd=2,
            padx=20,
            pady=10,
            command=self.on_exit
        )
        self.exit_button.pack(side='bottom', pady=30)
        
        # 在右侧添加数字显示区域
        numbers_title_font = font.Font(family="Arial", size=20, weight="bold")
        numbers_title = tk.Label(
            self.right_frame,
            text="/numbers 话题数据",
            font=numbers_title_font,
            bg='#434C5E',
            fg='#ECEFF4',
            pady=20
        )
        numbers_title.pack(pady=30)
        
        # 创建数字显示标签
        number_font = font.Font(family="Arial", size=72, weight="bold")
        self.number_label = tk.Label(
            self.right_frame,
            text="等待数据...",
            font=number_font,
            bg='#2E3440',
            fg='#88C0D0',
            relief='sunken',
            bd=3,
            padx=50,
            pady=50
        )
        self.number_label.pack(expand=True, fill='both', padx=30, pady=30)
        
        # 添加时间戳标签
        timestamp_font = font.Font(family="Arial", size=12)
        self.timestamp_label = tk.Label(
            self.right_frame,
            text="",
            font=timestamp_font,
            bg='#434C5E',
            fg='#D8DEE9'
        )
        self.timestamp_label.pack(pady=10)
        
        # 绑定键盘事件
        self.root.bind('<Escape>', lambda e: self.on_exit())
        self.root.bind('<Return>', lambda e: self.on_start_button_click())
        self.root.focus_set()
        
        # 启动GUI更新定时器
        self.root.after(100, self.process_gui_queue)
        
        self.get_logger().info('GUI界面已创建')
    
    def on_start_button_click(self):
        """启动按钮点击处理"""
        # 禁用按钮防止重复点击
        self.start_button.config(state='disabled')
        self.root.after(2000, lambda: self.start_button.config(state='normal'))
        
        # 调用服务
        threading.Thread(target=self.call_start_service, daemon=True).start()
    
    def on_exit(self):
        """退出应用"""
        self.get_logger().info('退出GUI应用')
        self.root.quit()
        self.root.destroy()
    
    def process_gui_queue(self):
        """处理GUI更新队列"""
        try:
            while True:
                action, data = self.gui_queue.get_nowait()
                
                if action == 'update_number':
                    # 更新数字显示
                    self.number_label.config(text=str(data))
                    
                    # 更新时间戳
                    import datetime
                    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    self.timestamp_label.config(text=f"最后更新: {timestamp}")
                    
                elif action == 'show_message':
                    # 更新状态信息
                    self.status_label.config(text=str(data))
                    
        except queue.Empty:
            pass
        
        # 继续处理队列
        self.root.after(100, self.process_gui_queue)
    
    def run(self):
        """运行GUI和ROS节点"""
        self.get_logger().info('DiffBot GUI节点启动')
        
        # 在单独线程中运行ROS spinning
        def spin_thread():
            try:
                rclpy.spin(self)
            except Exception as e:
                self.get_logger().error(f'ROS spinning错误: {e}')
        
        ros_thread = threading.Thread(target=spin_thread, daemon=True)
        ros_thread.start()
        
        try:
            # 运行GUI主循环
            self.root.mainloop()
        except KeyboardInterrupt:
            self.get_logger().info('接收到中断信号')
        finally:
            self.destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        gui_node = DiffBotGUI()
        gui_node.run()
    except Exception as e:
        print(f'错误: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
