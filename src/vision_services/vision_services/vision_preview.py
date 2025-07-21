"""
Preview interface for vision services
提供实时预览界面用于调试和可视化
"""
import cv2
import numpy as np
from typing import Optional, List, Tuple, Dict, Any
from geometry_msgs.msg import Point, Point32
import threading
import time


class VisionPreview:
    """视觉预览界面类"""
    
    def __init__(self, config: dict):
        """
        初始化预览界面
        
        Args:
            config: 配置字典
        """
        self.config = config
        self.preview_config = config.get('preview', {})
        self.debug_config = config.get('debug', {})
        
        self.enabled = self.preview_config.get('enabled', False)
        if not self.enabled:
            return
            
        # 显示选项
        self.display_options = self.preview_config.get('display_options', {})
        
        # 颜色配置
        self.colors = self.preview_config.get('colors', {
            'square_contour': [0, 255, 0],
            'rectangle_contour': [255, 0, 0],
            'center_point': [0, 0, 255],
            'corner_points': [255, 255, 0],
            'midline': [128, 0, 128],
            'text': [255, 255, 255]
        })
        
        # 绘制参数
        self.drawing = self.preview_config.get('drawing', {
            'contour_thickness': 2,
            'point_radius': 5,
            'text_font_scale': 0.6,
            'text_thickness': 2
        })
        
        # 窗口大小和位置
        self.window_size = self.preview_config.get('window_size', {'width': 800, 'height': 600})
        self.window_pos = self.preview_config.get('window_position', {'x': 100, 'y': 100})
        
        # 图像缓存
        self.original_image = None
        self.preprocessed_image = None
        self.result_image = None
        
        # 检测结果缓存
        self.detection_results = {
            'squares': [],
            'rectangles': [],
            'outer_rectangles': [],
            'centers': [],
            'corners': [],
            'midlines': [],
            'angles': []
        }
        
        # 状态信息
        self.status_info = {
            'last_detection': 'None',
            'detection_time': 0.0,
            'fps': 0.0,
            'image_size': (0, 0)
        }
        
        # 窗口创建标志
        self.windows_created = False
        
        # 线程控制
        self.update_thread = None
        self.running = False
        
        # FPS计算
        self.fps_counter = 0
        self.fps_start_time = time.time()
        
        if self.enabled:
            self._setup_windows()
            self._start_update_thread()
    
    def _setup_windows(self):
        """设置预览窗口"""
        if not self.enabled:
            return
            
        try:
            # 创建主窗口
            if self.display_options.get('show_original', True):
                cv2.namedWindow('Original Image', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Original Image', 
                               self.window_size['width']//2, 
                               self.window_size['height']//2)
                cv2.moveWindow('Original Image', 
                             self.window_pos['x'], 
                             self.window_pos['y'])
            
            if self.display_options.get('show_preprocessed', True):
                cv2.namedWindow('Preprocessed Image', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Preprocessed Image', 
                               self.window_size['width']//2, 
                               self.window_size['height']//2)
                cv2.moveWindow('Preprocessed Image', 
                             self.window_pos['x'] + self.window_size['width']//2, 
                             self.window_pos['y'])
            
            if self.display_options.get('show_results', True):
                cv2.namedWindow('Detection Results', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Detection Results', 
                               self.window_size['width'], 
                               self.window_size['height'])
                cv2.moveWindow('Detection Results', 
                             self.window_pos['x'], 
                             self.window_pos['y'] + self.window_size['height']//2)
            
            self.windows_created = True
            
        except Exception as e:
            print(f"Error setting up preview windows: {e}")
            self.enabled = False
    
    def _start_update_thread(self):
        """启动更新线程"""
        if not self.enabled:
            return
            
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()
    
    def _update_loop(self):
        """更新循环"""
        update_rate = self.preview_config.get('update_rate', 30)
        sleep_time = 1.0 / update_rate
        
        while self.running:
            try:
                if self.windows_created:
                    self._update_displays()
                time.sleep(sleep_time)
            except Exception as e:
                print(f"Error in preview update loop: {e}")
                break
    
    def _update_displays(self):
        """更新显示"""
        if not self.enabled or not self.windows_created:
            return
            
        # 更新原始图像
        if (self.display_options.get('show_original', True) and 
            self.original_image is not None):
            cv2.imshow('Original Image', self.original_image)
        
        # 更新预处理图像
        if (self.display_options.get('show_preprocessed', True) and 
            self.preprocessed_image is not None):
            if len(self.preprocessed_image.shape) == 2:
                # 将灰度图转换为彩色显示
                display_img = cv2.cvtColor(self.preprocessed_image, cv2.COLOR_GRAY2BGR)
            else:
                display_img = self.preprocessed_image
            cv2.imshow('Preprocessed Image', display_img)
        
        # 更新结果图像
        if (self.display_options.get('show_results', True) and 
            self.original_image is not None):
            result_img = self._create_result_image()
            cv2.imshow('Detection Results', result_img)
        
        cv2.waitKey(1)
    
    def _create_result_image(self) -> np.ndarray:
        """创建结果图像"""
        if self.original_image is None:
            return np.zeros((100, 100, 3), dtype=np.uint8)
        
        result_img = self.original_image.copy()
        
        # 绘制正方形
        for square in self.detection_results['squares']:
            cv2.drawContours(result_img, [square], -1, 
                           self.colors['square_contour'], 
                           self.drawing['contour_thickness'])
        
        # 绘制矩形
        for rectangle in self.detection_results['rectangles']:
            cv2.drawContours(result_img, [rectangle], -1, 
                           self.colors['rectangle_contour'], 
                           self.drawing['contour_thickness'])
        
        # 绘制外框（胶带外边界）
        for outer_rect in self.detection_results['outer_rectangles']:
            cv2.drawContours(result_img, [outer_rect], -1, 
                           [255, 255, 0],  # 黄色外框
                           self.drawing['contour_thickness'])
            # 绘制外框角点
            for i in range(len(outer_rect)):
                cv2.circle(result_img, 
                          (int(outer_rect[i][0]), int(outer_rect[i][1])), 
                          self.drawing['point_radius'], 
                          [255, 255, 0], -1)  # 黄色角点
        
        # 绘制中心点
        for center in self.detection_results['centers']:
            cv2.circle(result_img, 
                      (int(center.x), int(center.y)), 
                      self.drawing['point_radius'], 
                      self.colors['center_point'], -1)
        
        # 绘制角点
        for corner in self.detection_results['corners']:
            cv2.circle(result_img, 
                      (int(corner.x), int(corner.y)), 
                      self.drawing['point_radius']//2, 
                      self.colors['corner_points'], -1)
        
        # 绘制中线
        for midline in self.detection_results['midlines']:
            if len(midline) >= 4:
                # 绘制中线矩形
                midline_array = np.array([[int(p.x), int(p.y)] for p in midline[:4]], dtype=np.int32)
                cv2.drawContours(result_img, [midline_array], -1, 
                               self.colors['midline'], 
                               self.drawing['contour_thickness'])
                # 绘制中线角点
                for i in range(len(midline_array)):
                    cv2.circle(result_img, 
                              (int(midline_array[i][0]), int(midline_array[i][1])), 
                              self.drawing['point_radius']//2, 
                              self.colors['midline'], -1)  # 紫色角点
        
        # 绘制信息面板
        if self.display_options.get('show_info_panel', True):
            result_img = self._add_info_panel(result_img)
        
        return result_img
    
    def _add_info_panel(self, image: np.ndarray) -> np.ndarray:
        """添加信息面板"""
        panel_height = 120
        panel_width = image.shape[1]
        
        # 创建信息面板
        panel = np.zeros((panel_height, panel_width, 3), dtype=np.uint8)
        panel[:] = (50, 50, 50)  # 深灰色背景
        
        # 添加文本信息
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = self.drawing['text_font_scale']
        thickness = self.drawing['text_thickness']
        color = tuple(self.colors['text'])
        
        y_offset = 20
        line_height = 25
        
        # 更新FPS
        self._update_fps()
        
        texts = [
            f"Last Detection: {self.status_info['last_detection']}",
            f"Detection Time: {self.status_info['detection_time']:.3f}s",
            f"FPS: {self.status_info['fps']:.1f}",
            f"Image Size: {self.status_info['image_size']}"
        ]
        
        for i, text in enumerate(texts):
            y = y_offset + i * line_height
            cv2.putText(panel, text, (10, y), font, font_scale, color, thickness)
        
        # 将面板添加到图像底部
        combined = np.vstack([image, panel])
        return combined
    
    def _update_fps(self):
        """更新FPS计算"""
        self.fps_counter += 1
        current_time = time.time()
        elapsed = current_time - self.fps_start_time
        
        if elapsed >= 1.0:  # 每秒更新一次FPS
            self.status_info['fps'] = self.fps_counter / elapsed
            self.fps_counter = 0
            self.fps_start_time = current_time
    
    def update_original_image(self, image: np.ndarray):
        """更新原始图像"""
        if not self.enabled:
            return
            
        self.original_image = image.copy()
        if image is not None:
            self.status_info['image_size'] = (image.shape[1], image.shape[0])
    
    def update_preprocessed_image(self, image: np.ndarray):
        """更新预处理图像"""
        if not self.enabled:
            return
            
        self.preprocessed_image = image.copy()
    
    def update_square_detection(self, center: Optional[Point], corners: Optional[List[Point32]]):
        """更新正方形检测结果"""
        if not self.enabled:
            return
            
        self.detection_results['squares'] = []
        self.detection_results['centers'] = []
        self.detection_results['corners'] = []
        
        if center is not None:
            self.detection_results['centers'] = [center]
            self.status_info['last_detection'] = 'Square'
        
        if corners is not None:
            self.detection_results['corners'] = corners
            # 转换为OpenCV格式
            corners_array = np.array([[int(p.x), int(p.y)] for p in corners], dtype=np.int32)
            self.detection_results['squares'] = [corners_array]
    
    def update_rectangle_detection(self, midline_points: Optional[List[Point32]], 
                                 angle: Optional[float] = None,
                                 outer_corners: Optional[List[Point32]] = None):
        """更新矩形检测结果"""
        if not self.enabled:
            return
            
        self.detection_results['rectangles'] = []
        self.detection_results['midlines'] = []
        self.detection_results['angles'] = []
        self.detection_results['outer_rectangles'] = []
        
        if midline_points is not None:
            self.detection_results['midlines'] = [midline_points]
            # 转换为OpenCV格式
            midline_array = np.array([[int(p.x), int(p.y)] for p in midline_points[:4]], dtype=np.int32)
            self.detection_results['rectangles'] = [midline_array]
            
            # 添加外框显示
            if outer_corners is not None:
                outer_array = np.array([[int(p.x), int(p.y)] for p in outer_corners[:4]], dtype=np.int32)
                self.detection_results['outer_rectangles'] = [outer_array]
            
            if angle is not None:
                self.detection_results['angles'] = [angle]
                self.status_info['last_detection'] = f'Rectangle (angle: {angle:.1f}°)'
            else:
                self.status_info['last_detection'] = 'Rectangle'
    
    def set_detection_time(self, detection_time: float):
        """设置检测时间"""
        if not self.enabled:
            return
            
        self.status_info['detection_time'] = detection_time
    
    def cleanup(self):
        """清理资源"""
        if not self.enabled:
            return
            
        self.running = False
        if self.update_thread is not None:
            self.update_thread.join(timeout=1.0)
        
        try:
            cv2.destroyAllWindows()
        except:
            pass
    
    def __del__(self):
        """析构函数"""
        self.cleanup()
