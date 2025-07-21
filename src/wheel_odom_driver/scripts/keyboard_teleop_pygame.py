#!/usr/bin/env python3
"""
Pygame-based keyboard teleop node
- Control /cmd_vel (geometry_msgs/msg/Twist): linear.x, linear.y, angular.z
- Control /gimbal/angle_cmd (geometry_msgs/msg/Vector3): gimbal pitch, yaw
- Step size: 0.05, cumulative control
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import pygame
import sys
import threading
import time

STEP = 0.05  # Step size

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_pygame')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gimbal_pub = self.create_publisher(Vector3, '/gimbal/angle_cmd', 10)
        
        # Velocity control variables
        self.linear_x = 0.0   # Forward/backward
        self.linear_y = 0.0   # Left/right strafe
        self.angular_z = 0.0  # Turn left/right
        
        # Gimbal control variables (only using x and y for pitch and yaw)
        self.gimbal_pitch = 0.0  # Up/down (x axis)
        self.gimbal_yaw = 0.0    # Left/right (y axis)
        
        # Thread control
        self.running = True
        self.lock = threading.Lock()
        
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((600, 300))
        pygame.display.set_caption('ROS2 Keyboard Teleop (Pygame)')
        try:
            self.font = pygame.font.Font(None, 20)
        except:
            self.font = pygame.font.SysFont('arial', 16)
        
        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.publish_commands)
        
        # Log startup info
        self.get_logger().info("Keyboard teleop node started")
        self.get_logger().info("Controls: WASD=movement, ZX=turn, IJKL=gimbal, R=reset, ESC=quit")

    def publish_commands(self):
        with self.lock:
            # Publish cmd_vel
            twist = Twist()
            twist.linear.x = self.linear_x
            twist.linear.y = self.linear_y
            twist.angular.z = self.angular_z
            self.cmd_vel_pub.publish(twist)
            
            # Publish gimbal commands
            gimbal = Vector3()
            gimbal.x = self.gimbal_pitch  # Pitch (up/down)
            gimbal.y = self.gimbal_yaw    # Yaw (left/right)
            gimbal.z = 0.0                # Not used
            self.gimbal_pub.publish(gimbal)
        
        # Update display (call outside lock to avoid blocking)
        self.draw_status()

    def draw_status(self):
        # Clear screen with dark background
        self.screen.fill((20, 20, 20))
        
        # Status lines
        lines = [
            f'=== ROS2 Keyboard Teleop ===',
            f'',
            f'Movement Control:',
            f'  W/S: Forward/Backward  (linear.x={self.linear_x:.2f})',
            f'  A/D: Strafe Left/Right (linear.y={self.linear_y:.2f})',
            f'  Z/X: Turn Left/Right   (angular.z={self.angular_z:.2f})',
            f'',
            f'Gimbal Control:',
            f'  I/K: Pitch Up/Down     (pitch={self.gimbal_pitch:.2f})',
            f'  J/L: Yaw Left/Right    (yaw={self.gimbal_yaw:.2f})',
            f'',
            f'R: Reset all values',
            f'ESC/Q: Quit',
        ]
        
        for i, text in enumerate(lines):
            if text:  # Only render non-empty lines
                color = (255, 255, 255) if '===' in text else (200, 200, 200)
                img = self.font.render(text, True, color)
                self.screen.blit(img, (10, 10 + i * 18))
        
        pygame.display.flip()

    def run(self):
        self.get_logger().info("Pygame window opened. Use keyboard to control robot.")
        
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    self.handle_key(event.key)
            
            # Small delay to prevent high CPU usage
            pygame.time.wait(10)
        
        pygame.quit()
        self.get_logger().info("Keyboard teleop node stopped")
        sys.exit(0)

    def handle_key(self, key):
        with self.lock:
            if key == pygame.K_ESCAPE:
                self.running = False
            elif key == pygame.K_q:
                self.running = False
            # Movement control (cmd_vel)
            elif key == pygame.K_w:
                self.linear_x += STEP
                self.get_logger().info(f"Forward: linear.x = {self.linear_x:.2f}")
            elif key == pygame.K_s:
                self.linear_x -= STEP
                self.get_logger().info(f"Backward: linear.x = {self.linear_x:.2f}")
            elif key == pygame.K_a:
                self.linear_y += STEP
                self.get_logger().info(f"Strafe left: linear.y = {self.linear_y:.2f}")
            elif key == pygame.K_d:
                self.linear_y -= STEP
                self.get_logger().info(f"Strafe right: linear.y = {self.linear_y:.2f}")
            elif key == pygame.K_z:  # Turn left
                self.angular_z += STEP
                self.get_logger().info(f"Turn left: angular.z = {self.angular_z:.2f}")
            elif key == pygame.K_x:  # Turn right
                self.angular_z -= STEP
                self.get_logger().info(f"Turn right: angular.z = {self.angular_z:.2f}")
            # Gimbal control
            elif key == pygame.K_i:
                self.gimbal_pitch += STEP
                self.get_logger().info(f"Gimbal up: pitch = {self.gimbal_pitch:.2f}")
            elif key == pygame.K_k:
                self.gimbal_pitch -= STEP
                self.get_logger().info(f"Gimbal down: pitch = {self.gimbal_pitch:.2f}")
            elif key == pygame.K_j:
                self.gimbal_yaw += STEP
                self.get_logger().info(f"Gimbal left: yaw = {self.gimbal_yaw:.2f}")
            elif key == pygame.K_l:
                self.gimbal_yaw -= STEP
                self.get_logger().info(f"Gimbal right: yaw = {self.gimbal_yaw:.2f}")
            # Reset all values
            elif key == pygame.K_r:
                self.linear_x = 0.0
                self.linear_y = 0.0
                self.angular_z = 0.0
                self.gimbal_pitch = 0.0
                self.gimbal_yaw = 0.0
                self.get_logger().info("Reset all values to zero")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    
    # Start ROS2 spinning in a separate thread
    def spin_thread():
        rclpy.spin(node)
    
    ros_thread = threading.Thread(target=spin_thread, daemon=True)
    ros_thread.start()
    
    try:
        # Run pygame in main thread
        node.run()
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
