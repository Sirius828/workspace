#!/usr/bin/env python3

"""
Chassis Simulation Node

This node simulates the chassis hardware without serial communication.
It receives velocity commands and publishes odometry data, simulating
a three-omni-wheel chassis robot.

Features:
- Subscribes to /cmd_vel and /cmd_gimbal
- Publishes /odom and /tf
- Simulates robot movement and odometry integration
- No serial communication required - perfect for testing and debugging
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
import tf2_ros
from tf2_ros import TransformBroadcaster
import math
import tf_transformations

class ChassisSimulation(Node):
    def __init__(self):
        super().__init__('chassis_simulation')
        
        # Declare parameters
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_rate', 100.0)  # Hz - increased for smoother movement
        self.declare_parameter('enable_noise', False)  # Add noise to simulation
        self.declare_parameter('noise_std', 0.01)     # Standard deviation for noise
        
        # Get parameters
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_noise = self.get_parameter('enable_noise').value
        self.noise_std = self.get_parameter('noise_std').value
        
        # Robot state variables
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vw = 0.0
        self.current_gimbal_yaw = 0.0
        self.current_gimbal_pitch = 0.0
        self.current_victory_state = 0
        
        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()
        
        # Publishers
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscribers
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.cmd_gimbal_subscriber = self.create_subscription(
            Float32MultiArray, '/cmd_gimbal', self.cmd_gimbal_callback, 10)
        
        self.victory_subscriber = self.create_subscription(
            Int32, '/victory', self.victory_callback, 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing odometry
        self.timer = self.create_timer(1.0 / self.publish_rate, self.update_and_publish)
        
        self.get_logger().info('Chassis simulation node started - provides same visualization as hardware')
        self.get_logger().info(f'Publishing rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Base frame: {self.base_frame_id}, Odom frame: {self.odom_frame_id}')
        self.get_logger().info(f'Noise enabled: {self.enable_noise}')
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.current_vx = msg.linear.x
        self.current_vy = msg.linear.y
        self.current_vw = msg.angular.z
        
        self.get_logger().debug(f'Received cmd_vel: vx={self.current_vx:.3f}, vy={self.current_vy:.3f}, vw={self.current_vw:.3f}')
    
    def cmd_gimbal_callback(self, msg):
        """Handle gimbal commands"""
        if len(msg.data) >= 2:
            self.current_gimbal_yaw = msg.data[0]
            self.current_gimbal_pitch = msg.data[1]
            self.get_logger().debug(f'Received cmd_gimbal: yaw={self.current_gimbal_yaw:.3f}°, pitch={self.current_gimbal_pitch:.3f}°')
    
    def victory_callback(self, msg):
        """Handle victory state"""
        self.current_victory_state = msg.data
        self.get_logger().debug(f'Victory state updated: {self.current_victory_state}')
    
    def add_noise(self, value, std_dev):
        """Add Gaussian noise to a value"""
        if not self.enable_noise:
            return value
        import random
        return value + random.gauss(0, std_dev)
    
    def update_and_publish(self):
        """Update robot pose and publish odometry"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Simple integration of velocities (assuming perfect tracking)
        # For three-omni-wheel chassis, the robot can move in any direction
        
        # Convert body velocities to world frame
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        # World frame velocities
        world_vx = cos_yaw * self.current_vx - sin_yaw * self.current_vy
        world_vy = sin_yaw * self.current_vx + cos_yaw * self.current_vy
        world_vw = self.current_vw
        
        # Integrate position and orientation
        self.x += world_vx * dt
        self.y += world_vy * dt
        self.yaw += world_vw * dt
        
        # Normalize yaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        
        # Add noise if enabled
        noisy_x = self.add_noise(self.x, self.noise_std)
        noisy_y = self.add_noise(self.y, self.noise_std)
        noisy_yaw = self.add_noise(self.yaw, self.noise_std * 0.1)  # Less noise on angle
        noisy_vx = self.add_noise(self.current_vx, self.noise_std * 0.1)
        noisy_vy = self.add_noise(self.current_vy, self.noise_std * 0.1)
        noisy_vw = self.add_noise(self.current_vw, self.noise_std * 0.1)
        
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        
        # Position
        odom_msg.pose.pose.position.x = noisy_x
        odom_msg.pose.pose.position.y = noisy_y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (convert yaw to quaternion)
        quat = tf_transformations.quaternion_from_euler(0, 0, noisy_yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Velocity (in body frame)
        odom_msg.twist.twist.linear.x = noisy_vx
        odom_msg.twist.twist.linear.y = noisy_vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = noisy_vw
        
        # Covariance (matching hardware node)
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.twist.covariance = [0.0] * 36
        
        # Diagonal covariance values
        odom_msg.pose.covariance[0] = 0.1   # x
        odom_msg.pose.covariance[7] = 0.1   # y
        odom_msg.pose.covariance[35] = 0.1  # yaw
        odom_msg.twist.covariance[0] = 0.1  # vx
        odom_msg.twist.covariance[7] = 0.1  # vy
        odom_msg.twist.covariance[35] = 0.1 # vw
        
        # Publish odometry
        self.odom_publisher.publish(odom_msg)
        
        # Broadcast transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        
        transform.transform.translation.x = noisy_x
        transform.transform.translation.y = noisy_y
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(transform)
        
        # Log current state for visualization
        self.get_logger().debug(
            f'Position: x={noisy_x:.3f}m, y={noisy_y:.3f}m, yaw={math.degrees(noisy_yaw):.1f}°'
        )
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    
    node = ChassisSimulation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Chassis simulation stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
