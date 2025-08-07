import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):

    def __init__(self):
        # Initialize base Node explicitly to avoid Pylance super() warning
        Node.__init__(self, 'camera_publisher')
        # 声明可通过参数设置的摄像头设备和话题名
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('topic', '/camera/color/image_raw')
        device = self.get_parameter('video_device').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value

        # OpenCV 摄像头
        self.cap = cv2.VideoCapture(device,cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 400)
        # pipeline = (
        #     f"v4l2src device={device} ! "
        #     "video/x-raw,format=MJPG,width=640,height=480,framerate=120/1 ! "
        #     "videoconvert ! appsink"
        #     )
        # self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开摄像头: {device}')
            raise RuntimeError('Camera open failed')
        # cv2.namedWindow('camera_preview', cv2.WINDOW_NORMAL)

        self.pub = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        # 定时器：按 400Hz 读取并发布
        timer_period = 1.0 / 400
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f'Publishing images from {device} on "{topic}" at 400 Hz')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('读取摄像头帧失败')
            return
        # 转灰度，减少带宽与后端预处理
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('camera_preview', frame)
        # 转成 ROS Image 并发布
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='mono8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        self.pub.publish(msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
