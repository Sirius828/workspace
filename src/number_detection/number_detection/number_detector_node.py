#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from ultralytics import YOLO


class NumberDetectorNode(Node):
    def __init__(self):
        super().__init__('number_detector')
        
        # 参数声明
        self.declare_parameter('model_path', 'modules/numbers.engine')
        self.declare_parameter('confidence_threshold', 0.75)
        self.declare_parameter('device', 0)  # GPU设备号
        self.declare_parameter('enable_preview', True)  # 是否启用预览画面
        self.declare_parameter('min_number', 30)  # 最小数字
        self.declare_parameter('max_number', 60)  # 最大数字
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.device = self.get_parameter('device').value
        self.enable_preview = self.get_parameter('enable_preview').value
        self.min_number = self.get_parameter('min_number').value
        self.max_number = self.get_parameter('max_number').value
        
        # 构建模型完整路径
        # 首先尝试绝对路径
        if os.path.isabs(model_path):
            self.model_full_path = model_path
        else:
            # 尝试多个可能的路径
            possible_paths = [
                # 源码路径
                os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), model_path),
                # 安装路径
                os.path.join('/home/sirius/ssd/ros2workspace/install/number_detection/share/number_detection', model_path),
                # 直接使用相对路径
                model_path
            ]
            
            self.model_full_path = None
            for path in possible_paths:
                if os.path.exists(path):
                    self.model_full_path = path
                    break
            
            if self.model_full_path is None:
                # 如果都找不到，使用默认的源码路径
                self.model_full_path = os.path.join(
                    '/home/sirius/ssd/ros2workspace/src/number_detection', 
                    model_path
                )
        
        # 初始化YOLO模型
        self.get_logger().info(f'Loading YOLO model from: {self.model_full_path}')
        try:
            self.model = YOLO(self.model_full_path, task="detect")
            self.class_names = self.model.names
            self.get_logger().info(f'Model loaded successfully. Classes: {list(self.class_names.values())}')
            
            # 构建有效数字列表
            self.valid_numbers = []
            for cls_id, cls_name in self.class_names.items():
                try:
                    number = int(cls_name)
                    if self.min_number <= number <= self.max_number:
                        self.valid_numbers.append((cls_id, number))
                except ValueError:
                    # 如果类名不是数字，跳过
                    pass
            
            if self.valid_numbers:
                self.get_logger().info(f'Valid numbers to detect: {[num for _, num in self.valid_numbers]}')
            else:
                self.get_logger().warn(f'No valid numbers found in range {self.min_number}-{self.max_number}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # QoS配置
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 订阅者和发布者
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos
        )
        
        self.number_pub = self.create_publisher(
            Int32,
            '/number',
            qos
        )
        
        # 可选：发布检测结果图像
        if self.enable_preview:
            self.result_image_pub = self.create_publisher(
                Image,
                '/number_detection_result',
                qos
            )
        
        self.get_logger().info(f'Number detector node started (preview: {self.enable_preview})')
    
    def image_callback(self, msg):
        """图像回调函数：进行数字检测"""
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # YOLO推理
            results = self.model.predict(
                cv_image, 
                device=self.device, 
                verbose=False,
                conf=self.conf_threshold
            )
            
            # 提取检测结果
            if len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy()
                scores = results[0].boxes.conf.cpu().numpy()
                cls_ids = results[0].boxes.cls.cpu().numpy().astype(int)
                
                # 查找有效数字的最佳检测结果
                best_detection = self.find_best_number(boxes, scores, cls_ids)
                
                if best_detection is not None:
                    # 发布数字
                    self.publish_number(best_detection)
                
                # 可选：发布检测结果图像和本地预览
                if self.enable_preview:
                    result_image = self.draw_detections(cv_image, boxes, scores, cls_ids)
                    # 显示本地预览窗口
                    cv2.imshow('Number Detection Preview', result_image)
                    cv2.waitKey(1)
                    # 发布 ROS 话题
                    self.publish_result_image(result_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'Error in image processing: {e}')
    
    def find_best_number(self, boxes, scores, cls_ids):
        """查找有效数字的最佳检测结果"""
        if not self.valid_numbers:
            return None
        
        # 查找有效数字的检测结果
        valid_detections = []
        for i, cls_id in enumerate(cls_ids):
            for valid_cls_id, number in self.valid_numbers:
                if cls_id == valid_cls_id:
                    valid_detections.append((i, number, scores[i]))
                    break
        
        if not valid_detections:
            return None
        
        # 选择置信度最高的数字
        best_detection = max(valid_detections, key=lambda x: x[2])
        best_idx, best_number, best_score = best_detection
        
        return boxes[best_idx], best_score, cls_ids[best_idx], best_number
    
    def publish_number(self, detection):
        """发布检测到的数字"""
        box, score, cls_id, number = detection
        x1, y1, x2, y2 = box
        
        # 计算矩形中心
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        
        # 创建Int32消息
        number_msg = Int32()
        number_msg.data = number
        
        self.number_pub.publish(number_msg)
        
        self.get_logger().info(f'Number detected: {number} at ({center_x:.1f}, {center_y:.1f}) with confidence {score:.3f}')
    
    def draw_detections(self, img, boxes, scores, cls_ids):
        """在图像上绘制检测结果"""
        result_img = img.copy()
        
        for box, score, cls_id in zip(boxes, scores, cls_ids):
            if score < self.conf_threshold:
                continue
                
            x1, y1, x2, y2 = box.astype(int)
            class_name = self.class_names[int(cls_id)]
            label = f"{class_name}: {score:.2f}"
            
            # 检查是否为有效数字
            is_valid_number = False
            for valid_cls_id, _ in self.valid_numbers:
                if cls_id == valid_cls_id:
                    is_valid_number = True
                    break
            
            # 根据是否为有效数字选择颜色
            if is_valid_number:
                color = (0, 255, 0)  # 绿色：有效数字
            else:
                color = (0, 255, 255)  # 黄色：其他检测结果
            
            # 绘制边界框
            cv2.rectangle(result_img, (x1, y1), (x2, y2), color, 2)
            
            # 绘制标签背景
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(result_img, (x1, y1 - h - 8), (x1 + w, y1), color, -1)
            
            # 绘制标签文本
            cv2.putText(
                result_img,
                label,
                (x1, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )
            
            # 如果是有效数字，绘制中心点
            if is_valid_number:
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                cv2.circle(result_img, (center_x, center_y), 5, (0, 0, 255), -1)
        
        return result_img
    
    def publish_result_image(self, cv_image, header):
        """发布检测结果图像"""
        try:
            result_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            result_msg.header = header
            self.result_image_pub.publish(result_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing result image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = NumberDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
