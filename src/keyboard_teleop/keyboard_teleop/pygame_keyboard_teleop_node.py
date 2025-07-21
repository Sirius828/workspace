#!/usr/bin/env python3
"""
Pygame-based keyboard teleop node
Uses pygame to handle keyboard input, solving terminal input issues
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import time

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False


class PygameKeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('pygame_keyboard_teleop_node')
        
        if not PYGAME_AVAILABLE:
            self.get_logger().error('pygame not installed! Please run: pip install pygame')
            raise ImportError('pygame is required for this node')
        
        # Declare parameters
        self.declare_parameter('cmd_vel_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('gimbal_angle_topic', '/gimbal/angle_cmd')
        self.declare_parameter('step_size', 0.05)
        self.declare_parameter('gimbal_step_size', 0.5)  # 0.5 degrees
        self.declare_parameter('max_linear_vel', 2.0)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('max_gimbal_yaw', 90.0)    # ±90 degrees
        self.declare_parameter('max_gimbal_pitch', 25.0)  # ±25 degrees
        self.declare_parameter('publish_rate', 10.0)
        
        # Get parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.gimbal_angle_topic = self.get_parameter('gimbal_angle_topic').get_parameter_value().string_value
        self.step_size = self.get_parameter('step_size').get_parameter_value().double_value
        self.gimbal_step_size = self.get_parameter('gimbal_step_size').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.max_gimbal_yaw = self.get_parameter('max_gimbal_yaw').get_parameter_value().double_value
        self.max_gimbal_pitch = self.get_parameter('max_gimbal_pitch').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.gimbal_angle_pub = self.create_publisher(Vector3, self.gimbal_angle_topic, 10)
        
        # Current velocities
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # Current gimbal angles in degrees (yaw, pitch)
        self.current_gimbal_yaw = 0.0    # degrees, range: -90 to +90
        self.current_gimbal_pitch = 0.0  # degrees, range: -25 to +25
        
        # Key states
        self.keys_pressed = set()
        
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((500, 400))
        pygame.display.set_caption('ROS2 Keyboard Teleop & Gimbal Control')
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 20)
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info('Pygame keyboard teleop node started')
        self.get_logger().info('Please ensure pygame window is in focus')
        self.get_logger().info(f'Publishing cmd_vel to: {self.cmd_vel_topic}')
        self.get_logger().info(f'Publishing gimbal angles to: {self.gimbal_angle_topic}')
        
    def timer_callback(self):
        """Timer callback"""
        # Process pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info('Received quit signal')
                rclpy.shutdown()
                return
            elif event.type == pygame.KEYDOWN:
                self.process_keydown(event.key)
            elif event.type == pygame.KEYUP:
                self.process_keyup(event.key)
        
        # Update display
        self.update_display()
        
        # Publish velocities and gimbal angles
        self.publish_velocity()
        self.publish_gimbal_angles()
        
        # Control frame rate
        self.clock.tick(60)
    
    def process_keydown(self, key):
        """Process key press"""
        # Robot movement controls
        if key == pygame.K_w:
            self.current_linear_x = min(self.current_linear_x + self.step_size, self.max_linear_vel)
        elif key == pygame.K_s:
            self.current_linear_x = max(self.current_linear_x - self.step_size, -self.max_linear_vel)
        elif key == pygame.K_a:
            self.current_angular_z = min(self.current_angular_z + self.step_size, self.max_angular_vel)
        elif key == pygame.K_d:
            self.current_angular_z = max(self.current_angular_z - self.step_size, -self.max_angular_vel)
        elif key == pygame.K_q:
            self.current_linear_y = min(self.current_linear_y + self.step_size, self.max_linear_vel)
        elif key == pygame.K_e:
            self.current_linear_y = max(self.current_linear_y - self.step_size, -self.max_linear_vel)
        
        # Gimbal controls
        elif key == pygame.K_UP:
            self.current_gimbal_pitch = min(self.current_gimbal_pitch + self.gimbal_step_size, self.max_gimbal_pitch)
        elif key == pygame.K_DOWN:
            self.current_gimbal_pitch = max(self.current_gimbal_pitch - self.gimbal_step_size, -self.max_gimbal_pitch)
        elif key == pygame.K_LEFT:
            self.current_gimbal_yaw = min(self.current_gimbal_yaw + self.gimbal_step_size, self.max_gimbal_yaw)
        elif key == pygame.K_RIGHT:
            self.current_gimbal_yaw = max(self.current_gimbal_yaw - self.gimbal_step_size, -self.max_gimbal_yaw)
        
        # Reset/Stop controls
        elif key == pygame.K_SPACE:
            self.current_linear_x = 0.0
            self.current_linear_y = 0.0
            self.current_angular_z = 0.0
        elif key == pygame.K_r:
            self.current_linear_x = 0.0
            self.current_linear_y = 0.0
            self.current_angular_z = 0.0
        elif key == pygame.K_g:  # Reset gimbal
            self.current_gimbal_yaw = 0.0
            self.current_gimbal_pitch = 0.0
        elif key == pygame.K_ESCAPE:
            rclpy.shutdown()
    
    def process_keyup(self, key):
        """Process key release"""
        pass  # No need to handle key release in step mode
    
    def update_display(self):
        """Update pygame display"""
        # Clear screen
        self.screen.fill((0, 0, 0))
        
        # Display title
        title = self.font.render('ROS2 Keyboard Teleop & Gimbal Control', True, (255, 255, 255))
        self.screen.blit(title, (10, 10))
        
        # Display current velocities
        velocity_text = [
            f'Linear X: {self.current_linear_x:+.3f} m/s',
            f'Linear Y: {self.current_linear_y:+.3f} m/s',
            f'Angular Z: {self.current_angular_z:+.3f} rad/s'
        ]
        
        for i, text in enumerate(velocity_text):
            rendered = self.font.render(text, True, (0, 255, 0))
            self.screen.blit(rendered, (10, 40 + i * 25))
        
        # Display current gimbal angles
        gimbal_text = [
            f'Gimbal Yaw: {self.current_gimbal_yaw:+.1f} deg (±{self.max_gimbal_yaw:.0f}°)',
            f'Gimbal Pitch: {self.current_gimbal_pitch:+.1f} deg (±{self.max_gimbal_pitch:.0f}°)'
        ]
        
        for i, text in enumerate(gimbal_text):
            rendered = self.font.render(text, True, (255, 255, 0))
            self.screen.blit(rendered, (10, 125 + i * 25))
        
        # Display robot control instructions
        robot_controls = [
            'Robot Controls:',
            'W/S: Forward/Backward',
            'A/D: Turn Left/Right',
            'Q/E: Strafe Left/Right',
            'SPACE: Stop robot',
            'R: Reset robot'
        ]
        
        for i, text in enumerate(robot_controls):
            color = (200, 200, 200) if i == 0 else (150, 150, 150)
            rendered = self.font.render(text, True, color)
            self.screen.blit(rendered, (10, 180 + i * 20))
        
        # Display gimbal control instructions
        gimbal_controls = [
            'Gimbal Controls:',
            f'Arrow Keys: Pan/Tilt ({self.gimbal_step_size:.1f}° step)',
            'G: Reset gimbal',
            'ESC: Exit'
        ]
        
        for i, text in enumerate(gimbal_controls):
            color = (200, 200, 200) if i == 0 else (150, 150, 150)
            rendered = self.font.render(text, True, color)
            self.screen.blit(rendered, (250, 180 + i * 20))
        
        # Update display
        pygame.display.flip()
    
    def publish_velocity(self):
        """Publish velocity commands"""
        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.linear.y = self.current_linear_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.current_angular_z
        
        self.cmd_vel_pub.publish(twist)
    
    def publish_gimbal_angles(self):
        """Publish gimbal angle commands in degrees"""
        gimbal_angles = Vector3()
        # Publish angles in degrees (not radians)
        gimbal_angles.x = self.current_gimbal_yaw   # Yaw in degrees
        gimbal_angles.y = self.current_gimbal_pitch # Pitch in degrees
        gimbal_angles.z = 0.0                       # Roll (unused)
        
        self.gimbal_angle_pub.publish(gimbal_angles)
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        # Send stop commands
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        gimbal_angles = Vector3()
        self.gimbal_angle_pub.publish(gimbal_angles)
        
        # Cleanup pygame
        if PYGAME_AVAILABLE:
            pygame.quit()
        
        self.get_logger().info('Pygame keyboard teleop node stopped')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = PygameKeyboardTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Program interrupted")
    except ImportError as e:
        print(f"Import error: {e}")
        print("Please install pygame: pip install pygame")
    except Exception as e:
        print(f"Program error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
