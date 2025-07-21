#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 参数声明
        self.declare_parameter('model_path', 'modules/ferrari.engine')
        self.declare_parameter('confidence_threshold', 0.85)
        self.declare_parameter('target_class', 'ferrari')  # 检测ferrari
        self.declare_parameter('device', 0)  # GPU设备号
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.target_class = self.get_parameter('target_class').value
        self.device = self.get_parameter('device').value
        
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
                os.path.join('/home/sirius/ssd/ros2workspace/install/yolo_detector/share/yolo_detector', model_path),
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
                    '/home/sirius/ssd/ros2workspace/src/yolo_detector', 
                    model_path
                )
        
        # 初始化YOLO模型
        self.get_logger().info(f'Loading YOLO model from: {self.model_full_path}')
        try:
            self.model = YOLO(self.model_full_path, task="detect")
            self.class_names = self.model.names
            self.get_logger().info(f'Model loaded successfully. Classes: {list(self.class_names.values())}')
            
            # 查找目标类别ID
            self.target_class_id = None
            for cls_id, cls_name in self.class_names.items():
                if cls_name.lower() == self.target_class.lower():
                    self.target_class_id = cls_id
                    break
            
            if self.target_class_id is not None:
                self.get_logger().info(f'Target class "{self.target_class}" found with ID: {self.target_class_id}')
            else:
                self.get_logger().warn(f'Target class "{self.target_class}" not found in model classes')
                
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # QoS配置
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 订阅者和发布者
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.image_callback,
            qos
        )
        
        self.position_pub = self.create_publisher(
            Point,
            '/target_position_pixel',
            qos
        )
        
        # 可选：发布检测结果图像
        self.result_image_pub = self.create_publisher(
            Image,
            '/yolo_detection_result',
            qos
        )
        
        self.get_logger().info('YOLO detector node started')
    
    def image_callback(self, msg):
        """图像回调函数：进行目标检测"""
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
                
                # 查找目标类别的最佳检测结果
                best_detection = self.find_best_target(boxes, scores, cls_ids)
                
                if best_detection is not None:
                    # 发布目标位置
                    self.publish_target_position(best_detection)
                
                # 可选：发布可视化结果
                result_image = self.draw_detections(cv_image, boxes, scores, cls_ids)
                self.publish_result_image(result_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'Error in image processing: {e}')
    
    def find_best_target(self, boxes, scores, cls_ids):
        """查找目标类别的最佳检测结果"""
        if self.target_class_id is None:
            # 如果没有指定目标类别，返回置信度最高的检测结果
            if len(scores) > 0:
                best_idx = np.argmax(scores)
                return boxes[best_idx], scores[best_idx], cls_ids[best_idx]
            return None
        
        # 查找目标类别的检测结果
        target_indices = np.where(cls_ids == self.target_class_id)[0]
        
        if len(target_indices) == 0:
            return None
        
        # 选择置信度最高的目标
        target_scores = scores[target_indices]
        best_target_idx = target_indices[np.argmax(target_scores)]
        
        return boxes[best_target_idx], scores[best_target_idx], cls_ids[best_target_idx]
    
    def publish_target_position(self, detection):
        """发布目标中心位置"""
        box, score, cls_id = detection
        x1, y1, x2, y2 = box
        
        # 计算矩形中心
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        
        # 创建Point消息
        point_msg = Point()
        point_msg.x = float(center_x)
        point_msg.y = float(center_y)
        point_msg.z = float(score)  # 将置信度存储在z坐标中
        
        self.position_pub.publish(point_msg)
        
        class_name = self.class_names[cls_id]
        self.get_logger().info(f'Target detected: {class_name} at ({center_x:.1f}, {center_y:.1f}) with confidence {score:.3f}')
    
    def draw_detections(self, img, boxes, scores, cls_ids):
        """在图像上绘制检测结果"""
        result_img = img.copy()
        
        for box, score, cls_id in zip(boxes, scores, cls_ids):
            if score < self.conf_threshold:
                continue
                
            x1, y1, x2, y2 = box.astype(int)
            class_name = self.class_names[int(cls_id)]
            label = f"{class_name}: {score:.2f}"
            
            # 根据是否为目标类别选择颜色
            if self.target_class_id is None or cls_id == self.target_class_id:
                color = (0, 255, 0)  # 绿色：目标类别
            else:
                color = (0, 255, 255)  # 黄色：其他类别
            
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
            
            # 如果是目标类别，绘制中心点
            if self.target_class_id is None or cls_id == self.target_class_id:
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
    node = YoloDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
