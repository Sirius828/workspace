"""
Image processing utilities for vision services
"""
import cv2
import numpy as np
import time
from typing import List, Tuple, Optional
from geometry_msgs.msg import Point, Point32
from .vision_preview import VisionPreview


class ImageProcessor:
    """图像处理类，包含各种检测算法"""
    
    def __init__(self, config: dict):
        """
        初始化图像处理器
        
        Args:
            config: 配置字典
        """
        self.config = config
        self.debug = config.get('debug', {}).get('show_images', False)
        
        # 初始化预览界面
        self.preview = VisionPreview(config)
        
        # 性能统计
        self.detection_times = {
            'square': [],
            'a4': [],
            'a4_rot': []
        }
    
    def preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """
        图像预处理
        
        Args:
            image: 输入图像
            
        Returns:
            预处理后的图像
        """
        # 应用中心裁切（如果启用）
        crop_config = self.config.get('image_preprocessing', {}).get('center_crop', {})
        if crop_config.get('enabled', False):
            image = self._apply_center_crop(image, crop_config)
         # 转换为灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        # 应用 ROI 掩码（如果启用）
        roi_config = self.config.get('image_preprocessing', {}).get('roi_mask', {})
        if roi_config.get('enabled', False):
            polygon = roi_config.get('polygon', [])
            if len(polygon) >= 3:  # 至少需要3个点
                # 创建掩码
                mask = np.zeros_like(gray, dtype=np.uint8)
                
                # 将扁平数组转换为点数组
                points = np.array(polygon).reshape(-1, 2).astype(np.int32)
                
                # 填充多边形区域
                cv2.fillPoly(mask, [points], 255)
                
                # 应用掩码，只保留 ROI 区域
                gray = cv2.bitwise_and(gray, mask)
                
                # print(f"Applied ROI mask with {len(points)} points")

        # 双边滤波（保留边缘和直角）
        blur_config = self.config['image_preprocessing']['gaussian_blur']
        # diameter for filter kernel
        d = max(blur_config['kernel_size_x'], blur_config['kernel_size_y'])
        sigma_color = blur_config['sigma_x'] * 100
        sigma_space = blur_config['sigma_y'] * 100
        gray = cv2.bilateralFilter(gray, d=d,
                                   sigmaColor=sigma_color,
                                   sigmaSpace=sigma_space)
        
        # 形态学闭合，使用矩形内核以保持直角
        morph_config = self.config['image_preprocessing']['morphology']
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,
                                           (morph_config['kernel_size_x'],
                                            morph_config['kernel_size_y']))
        gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel,
                                iterations=morph_config['iterations'])
        
        # 掩模增强处理（专门针对细铅笔线）
        if self.config['image_preprocessing'].get('mask_enhancement_enabled', False):
            gray = self._enhance_pencil_lines(gray)
        
        # Canny边缘检测
        canny_config = self.config['image_preprocessing']['canny']
        edges = cv2.Canny(gray, 
                         canny_config['low_threshold'], 
                         canny_config['high_threshold'],
                         apertureSize=canny_config['aperture_size'])
        
        # 线条膨胀增强（让细线条更粗）
        if self.config['image_preprocessing'].get('mask_enhancement_enabled', False):
            edges = self._dilate_pencil_lines(edges)
        
        return edges
    
    def _apply_center_crop(self, image: np.ndarray, crop_config: dict) -> np.ndarray:
        """
        应用中心裁切
        
        Args:
            image: 输入图像
            crop_config: 裁切配置
            
        Returns:
            裁切后的图像
        """
        h, w = image.shape[:2]
        
        # 获取配置参数
        crop_width = crop_config.get('width', None)
        crop_height = crop_config.get('height', None) 
        zoom_factor = crop_config.get('zoom_factor', 1.0)
        
        # 如果指定了具体尺寸，使用具体尺寸
        if crop_width and crop_height:
            target_w = min(crop_width, w)
            target_h = min(crop_height, h)
        # 否则使用缩放因子
        elif zoom_factor > 1.0:
            target_w = int(w / zoom_factor)
            target_h = int(h / zoom_factor)
        else:
            return image
        
        # 计算中心点
        center_y, center_x = h // 2, w // 2
        
        # 计算裁剪起始点
        start_x = max(0, center_x - target_w // 2)
        start_y = max(0, center_y - target_h // 2)
        end_x = min(w, start_x + target_w)
        end_y = min(h, start_y + target_h)
        
        # 裁剪图像
        cropped = image[start_y:end_y, start_x:end_x]
        
        return cropped
    
    def detect_square(self, image: np.ndarray) -> Tuple[Optional[Point], Optional[List[Point32]]]:
        """
        检测正方形框
        
        Args:
            image: 输入图像
            
        Returns:
            (中心点, 四个角点)
        """
        start_time = time.time()
        
        # 图像预处理（包含裁切）
        edges = self.preprocess_image(image)
        
        # 查找轮廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        square_config = self.config['square_detection']
        min_area = square_config['contour']['min_area']
        max_area = square_config['contour']['max_area']
        epsilon_factor = square_config['contour']['approx_epsilon_factor']
        
        squares = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if min_area <= area <= max_area:
                epsilon = epsilon_factor * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                if len(approx) == 4:
                    if self._is_square(approx, square_config['square_validation']):
                        squares.append(approx)
        
        if squares:
            # 选择面积最大的正方形
            best_square = max(squares, key=cv2.contourArea)
            
            # 计算中心点
            center = self._calculate_center(best_square)
            
            # 转换角点为Point32格式
            corners = []
            for point in best_square:
                corners.append(Point32(x=float(point[0][0]), y=float(point[0][1]), z=0.0))
            
            # 更新预览界面
            self.preview.update_square_detection(center, corners)
            
            detection_time = time.time() - start_time
            self.detection_times['square'].append(detection_time)
            self.preview.set_detection_time(detection_time)
            
            return center, corners
        
        # 更新预览界面（无检测结果）
        self.preview.update_square_detection(None, None)
        
        detection_time = time.time() - start_time
        self.detection_times['square'].append(detection_time)
        self.preview.set_detection_time(detection_time)
        
        return None, None
    
    def detect_a4_rectangle(self, image: np.ndarray, 
                           rotated: bool = False) -> Tuple[Optional[List[Point32]], Optional[float]]:
        """
        检测A4矩形框（胶带中线）
        
        Args:
            image: 输入图像
            rotated: 是否为旋转矩形
            
        Returns:
            (中线角点, 旋转角度)
        """
        start_time = time.time()
        
        # 应用中心裁切（如果启用）
        crop_config = self.config.get('image_preprocessing', {}).get('center_crop', {})
        if crop_config.get('enabled', False):
            processed_image = self._apply_center_crop(image, crop_config)
        else:
            processed_image = image

        # Apply ROI mask if enabled
        # roi_cfg = self.config.get('image_preprocessing', {}).get('roi_mask', {})
        # if roi_cfg.get('enabled', False):
        #     # build mask for color image
        #     mask = np.zeros(processed_image.shape[:2], dtype=np.uint8)
        #     raw_poly = roi_cfg.get('polygon', [])
        #     arr = np.array(raw_poly, dtype=np.int32)
        #     if arr.ndim == 1:
        #         pts = arr.reshape(-1, 2)
        #     else:
        #         pts = arr
        #     cv2.fillPoly(mask, [pts], 255)
        #     processed_image = cv2.bitwise_and(processed_image, processed_image, mask=mask)

        # 使用HSV颜色空间检测黑色胶带
        hsv = cv2.cvtColor(processed_image, cv2.COLOR_BGR2HSV)
        
        tape_config = self.config['a4_detection']['tape_detection']
        lower_bound = np.array([tape_config['hsv_lower_h'], tape_config['hsv_lower_s'], tape_config['hsv_lower_v']])
        upper_bound = np.array([tape_config['hsv_upper_h'], tape_config['hsv_upper_s'], tape_config['hsv_upper_v']])
        
        # 创建掩码
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # 形态学操作
        kernel_size = (tape_config['morphology_kernel_size_x'], tape_config['morphology_kernel_size_y'])
        kernel = np.ones(kernel_size, np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                               iterations=tape_config['morphology_iterations'])
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 更新预览界面的预处理图像
        self.preview.update_preprocessed_image(mask)
        
        print(f"Found {len(contours)} contours in mask")
        
        a4_config = self.config['a4_detection']
        min_area = a4_config['contour']['min_area']
        max_area = a4_config['contour']['max_area']
        
        rectangles = []
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            print(f"Contour {i}: area={area}")
            
            if min_area <= area <= max_area:
                print(f"Contour {i} passed area filter")
                
                if rotated:
                    # 最小外接矩形
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    
                    print(f"MinAreaRect angle: {rect[2]}")
                    
                    # 改进的旋转矩形验证
                    if self._is_valid_rotated_rectangle(contour, rect, a4_config):
                        print(f"Contour {i} is valid rotated rectangle")
                        rectangles.append((box, rect[2]))  # (角点, 角度)
                    else:
                        print(f"Contour {i} failed rotated rectangle validation")
                else:
                    # 多边形逼近
                    epsilon = a4_config['contour']['approx_epsilon_factor'] * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    
                    print(f"Polygon approximation: {len(approx)} points")
                    if len(approx) == 4:
                        # 重新排列为矩形格式
                        approx_rect = approx.reshape(-1, 2)
                        if self._is_valid_rectangle(approx_rect, a4_config['rectangle_validation']):
                            print(f"Contour {i} is valid rectangle")
                            rectangles.append((approx_rect, 0.0))
                        else:
                            print(f"Contour {i} failed rectangle validation")
                    else:
                        print(f"Contour {i} is not 4-sided polygon")
            else:
                print(f"Contour {i} failed area filter: {area} not in [{min_area}, {max_area}]")
        
        if rectangles:
            # 选择面积最大的矩形
            best_rect = max(rectangles, key=lambda x: cv2.contourArea(x[0]))
            corners, angle = best_rect
            
            # 计算胶带中线矩形
            midline_corners = self._calculate_midline_rectangle(corners)
            
            # 转换为Point32格式
            midline_points = []
            for point in midline_corners:
                midline_points.append(Point32(x=float(point[0]), y=float(point[1]), z=0.0))
            
            # 转换外框角点为Point32格式
            outer_points = []
            for point in corners:
                outer_points.append(Point32(x=float(point[0]), y=float(point[1]), z=0.0))
            
            # 更新预览界面，传递外框信息
            self.preview.update_rectangle_detection(midline_points, angle, outer_points)
            
            detection_time = time.time() - start_time
            self.detection_times['a4_rot' if rotated else 'a4'].append(detection_time)
            self.preview.set_detection_time(detection_time)
            
            return midline_points, angle
        
        # 更新预览界面（无检测结果）
        self.preview.update_rectangle_detection(None, None)
        
        detection_time = time.time() - start_time
        self.detection_times['a4_rot' if rotated else 'a4'].append(detection_time)
        self.preview.set_detection_time(detection_time)
        
        return None, None
    
    def update_preview(self, image: np.ndarray):
        """
        更新预览窗口显示实时图像流
        
        Args:
            image: 输入图像
        """
        if not self.config.get('preview', {}).get('enabled', False):
            return
            
        try:
            # 应用中心裁切（如果启用）到原始图像
            crop_config = self.config.get('image_preprocessing', {}).get('center_crop', {})
            if crop_config.get('enabled', False):
                cropped_image = self._apply_center_crop(image, crop_config)
            else:
                cropped_image = image
            
            # 更新原始图像显示（显示裁切后的图像）
            self.preview.update_original_image(cropped_image)
            
            # 可选：也显示预处理后的图像
            if self.config['preview']['display_options'].get('show_preprocessed', False):
                processed = self.preprocess_image(image)
                self.preview.update_preprocessed_image(processed)
            
        except Exception as e:
            # 静默处理错误，避免影响主要功能
            pass
    
    def _is_square(self, contour: np.ndarray, validation_config: dict) -> bool:
        """验证是否为正方形"""
        # 计算边长
        side_lengths = []
        for i in range(4):
            p1 = contour[i][0]
            p2 = contour[(i + 1) % 4][0]
            length = np.linalg.norm(p1 - p2)
            side_lengths.append(length)
        
        # 检查边长比例
        min_length = min(side_lengths)
        max_length = max(side_lengths)
        side_ratio = max_length / min_length
        
        min_ratio = validation_config['min_side_ratio']
        max_ratio = validation_config['max_side_ratio']
        
        if not (min_ratio <= side_ratio <= max_ratio):
            return False
        
        # 新的角度检测：寻找连续三个直角
        return self._has_three_consecutive_right_angles(contour, validation_config)
    
    def _has_three_consecutive_right_angles(self, contour: np.ndarray, validation_config: dict) -> bool:
        """
        检测是否有连续三个直角（适用于不完整的正方形）
        
        Args:
            contour: 四边形轮廓点
            validation_config: 验证配置
            
        Returns:
            是否检测到连续三个直角
        """
        min_angle = validation_config['min_angle']
        max_angle = validation_config['max_angle']
        
        # 计算所有四个角度
        angles = []
        for i in range(4):
            p1 = contour[i][0]
            p2 = contour[(i + 1) % 4][0]
            p3 = contour[(i + 2) % 4][0]
            
            v1 = p1 - p2
            v2 = p3 - p2
            
            # 防止除零错误
            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)
            if norm1 == 0 or norm2 == 0:
                angles.append(0)
                continue
            
            # 计算角度
            cos_angle = np.dot(v1, v2) / (norm1 * norm2)
            # 限制cos_angle在[-1, 1]范围内，避免数值误差
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            angle = np.arccos(cos_angle)
            angle_deg = np.degrees(angle)
            angles.append(angle_deg)
        
        # 检测连续三个直角的模式
        valid_angles = []
        for angle in angles:
            is_right_angle = min_angle <= angle <= max_angle
            valid_angles.append(is_right_angle)
        
        print(f"Angles: {[f'{a:.1f}°' for a in angles]}")
        print(f"Valid angles: {valid_angles}")
        
        # 检查是否有连续三个直角
        # 模式1: 位置0,1,2连续
        if valid_angles[0] and valid_angles[1] and valid_angles[2]:
            print("Found 3 consecutive right angles at positions 0,1,2")
            return True
        
        # 模式2: 位置1,2,3连续  
        if valid_angles[1] and valid_angles[2] and valid_angles[3]:
            print("Found 3 consecutive right angles at positions 1,2,3")
            return True
        
        # 模式3: 位置2,3,0连续（环形）
        if valid_angles[2] and valid_angles[3] and valid_angles[0]:
            print("Found 3 consecutive right angles at positions 2,3,0")
            return True
        
        # 模式4: 位置3,0,1连续（环形）
        if valid_angles[3] and valid_angles[0] and valid_angles[1]:
            print("Found 3 consecutive right angles at positions 3,0,1")
            return True
        
        # 如果没有找到连续三个直角，检查是否有至少3个直角（不要求连续）
        right_angle_count = sum(valid_angles)
        if right_angle_count >= 3:
            print(f"Found {right_angle_count} right angles (not necessarily consecutive)")
            return True
        
        print(f"Only found {right_angle_count} right angles, not enough for square detection")
        return False
    
    def _is_valid_rectangle(self, contour: np.ndarray, validation_config: dict) -> bool:
        """验证是否为有效矩形"""
        try:
            # 计算宽高比
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = max(w, h) / min(w, h)
            
            min_ratio = validation_config['min_aspect_ratio']
            max_ratio = validation_config['max_aspect_ratio']
            
            aspect_valid = min_ratio <= aspect_ratio <= max_ratio
            print(f"Rectangle validation: aspect_ratio={aspect_ratio:.2f}, valid={aspect_valid}")
            
            return aspect_valid
            
        except Exception as e:
            print(f"Rectangle validation error: {e}")
            return False
    
    def _is_valid_rotated_rectangle(self, contour: np.ndarray, rect: tuple, a4_config: dict) -> bool:
        """
        改进的旋转矩形验证方法
        
        Args:
            contour: 原始轮廓
            rect: minAreaRect返回的矩形信息 (center, (width, height), angle)
            a4_config: A4检测配置
            
        Returns:
            是否为有效的旋转矩形
        """
        try:
            center, (width, height), angle = rect
            
            # 1. 宽高比验证
            validation_config = a4_config['rectangle_validation']
            aspect_ratio = max(width, height) / min(width, height)
            min_ratio = validation_config['min_aspect_ratio']
            max_ratio = validation_config['max_aspect_ratio']
            
            if not (min_ratio <= aspect_ratio <= max_ratio):
                print(f"Failed aspect ratio: {aspect_ratio:.2f} not in [{min_ratio}, {max_ratio}]")
                return False
            
            # 2. 轮廓填充度验证（减少环境杂物）
            rect_area = width * height
            contour_area = cv2.contourArea(contour)
            fill_ratio = contour_area / rect_area if rect_area > 0 else 0
            
            min_fill_ratio = 0.7  # 至少70%的填充度
            if fill_ratio < min_fill_ratio:
                print(f"Failed fill ratio: {fill_ratio:.2f} < {min_fill_ratio}")
                return False
            
            # 3. 轮廓复杂度验证（排除复杂形状）
            perimeter = cv2.arcLength(contour, True)
            rect_perimeter = 2 * (width + height)
            complexity_ratio = perimeter / rect_perimeter if rect_perimeter > 0 else float('inf')
            
            max_complexity = 1.5  # 周长比不应超过1.5倍
            if complexity_ratio > max_complexity:
                print(f"Failed complexity: {complexity_ratio:.2f} > {max_complexity}")
                return False
            
            # 4. 凸包验证（确保形状相对规整）
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            convexity = contour_area / hull_area if hull_area > 0 else 0
            
            min_convexity = 0.85  # 至少85%的凸性
            if convexity < min_convexity:
                print(f"Failed convexity: {convexity:.2f} < {min_convexity}")
                return False
            
            print(f"Rotated rectangle validation passed: aspect={aspect_ratio:.2f}, fill={fill_ratio:.2f}, complexity={complexity_ratio:.2f}, convexity={convexity:.2f}")
            return True
            
        except Exception as e:
            print(f"Rotated rectangle validation error: {e}")
            return False
    
    def _calculate_center(self, contour: np.ndarray) -> Point:
        """计算轮廓中心点"""
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx = cy = 0
        
        return Point(x=float(cx), y=float(cy), z=0.0)
    
    def _calculate_midline_rectangle(self, outer_corners: np.ndarray) -> np.ndarray:
        """
        计算胶带中线矩形
        这里简化处理，假设胶带宽度为固定值
        实际应用中需要检测内外边界
        """
        # 简化实现：向内收缩固定像素
        shrink_pixels = 10
        
        # 计算轮廓中心
        center_x = np.mean(outer_corners[:, 0])
        center_y = np.mean(outer_corners[:, 1])
        
        # 向中心收缩
        midline_corners = []
        for corner in outer_corners:
            # 计算从中心到角点的向量
            dx = corner[0] - center_x
            dy = corner[1] - center_y
            
            # 减少距离
            length = np.sqrt(dx*dx + dy*dy)
            if length > 0:
                new_length = max(1, length - shrink_pixels)
                ratio = new_length / length
                new_x = center_x + dx * ratio
                new_y = center_y + dy * ratio
            else:
                new_x, new_y = corner[0], corner[1]
            
            midline_corners.append([new_x, new_y])
        
        return np.array(midline_corners)
    
    def cleanup(self):
        """清理资源"""
        if hasattr(self, 'preview'):
            self.preview.cleanup()
    
    def get_performance_stats(self) -> dict:
        """获取性能统计"""
        stats = {}
        for detection_type, times in self.detection_times.items():
            if times:
                stats[detection_type] = {
                    'count': len(times),
                    'avg_time': sum(times) / len(times),
                    'min_time': min(times),
                    'max_time': max(times)
                }
        return stats
