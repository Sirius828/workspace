#!/usr/bin/env python3

"""
Chassis Visualization Node

This node provides visualization for the chassis robot in RViz2.
It subscribes to odometry and other robot topics to display the robot state.

Features:
- Robot model: Cylinder with radius 110.737mm, height 70mm
- Subscribes to /odom for robot position
- Subscribes to /cmd_vel for velocity visualization
- Publishes robot_description and visualization markers
- Real-time visualization in RViz2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Int32
import math
import tf_transformations

class ChassisVisualization(Node):
    def __init__(self):
        super().__init__('chassis_visualization')
        
        # Robot parameters (convert mm to meters)
        self.robot_radius = 0.110737  # 110.737mm = 0.110737m
        self.robot_height = 0.070     # 70mm = 0.070m
        
        # Robot state
        self.current_pose = None
        self.current_velocity = None
        self.victory_state = False
        
        # Publishers
        self.robot_description_publisher = self.create_publisher(
            String, '/robot_description', 10)
        self.marker_publisher = self.create_publisher(
            Marker, '/visualization_marker', 10)
        
        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.victory_subscriber = self.create_subscription(
            Int32, '/victory', self.victory_callback, 10)
        
        # Timers
        self.description_timer = self.create_timer(1.0, self.publish_robot_description)
        self.marker_timer = self.create_timer(0.1, self.publish_markers)  # 10Hz
        
        self.get_logger().info('Chassis Visualization Node started')
        self.get_logger().info(f'Robot model: Cylinder radius={self.robot_radius*1000:.1f}mm, height={self.robot_height*1000:.1f}mm')
        self.get_logger().info('Subscribing to: /odom, /cmd_vel, /victory')
        self.get_logger().info('Publishing to: /robot_description, /visualization_marker')
    
    def generate_urdf(self):
        """Generate URDF description for the robot"""
        urdf_content = f"""<?xml version="1.0"?>
<robot name="chassis_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 {self.robot_height/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="{self.robot_radius}" length="{self.robot_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.4 0.8 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 {self.robot_height/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="{self.robot_radius}" length="{self.robot_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 {self.robot_height/2}" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Direction indicator (small cylinder on front) -->
  <link name="direction_indicator">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.01" length="0.02"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
  </link>
  
  <joint name="direction_joint" type="fixed">
    <parent link="base_link"/>
    <child link="direction_indicator"/>
    <origin xyz="{self.robot_radius-0.01} 0 {self.robot_height + 0.01}" rpy="0 0 0"/>
  </joint>
</robot>"""
        return urdf_content
    
    def publish_robot_description(self):
        """Publish robot description"""
        urdf_string = self.generate_urdf()
        msg = String()
        msg.data = urdf_string
        self.robot_description_publisher.publish(msg)
    
    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.current_pose = msg.pose.pose
        self.get_logger().debug('Received odometry update')
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands for visualization"""
        self.current_velocity = msg
        self.get_logger().debug(f'Velocity for visualization: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, vw={msg.angular.z:.3f}')
    
    def victory_callback(self, msg):
        """Handle victory state updates"""
        self.victory_state = bool(msg.data)
        self.get_logger().debug(f'Victory state updated: {self.victory_state}')
    
    def publish_markers(self):
        """Publish visualization markers"""
        if self.current_pose is None:
            return
            
        # Main robot body marker
        robot_marker = Marker()
        robot_marker.header.frame_id = "base_link"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "chassis_robot"
        robot_marker.id = 0
        robot_marker.type = Marker.CYLINDER
        robot_marker.action = Marker.ADD
        
        # Position (relative to base_link)
        robot_marker.pose.position.x = 0.0
        robot_marker.pose.position.y = 0.0
        robot_marker.pose.position.z = self.robot_height / 2
        
        # Orientation
        robot_marker.pose.orientation.x = 0.0
        robot_marker.pose.orientation.y = 0.0
        robot_marker.pose.orientation.z = 0.0
        robot_marker.pose.orientation.w = 1.0
        
        # Scale
        robot_marker.scale.x = self.robot_radius * 2  # diameter
        robot_marker.scale.y = self.robot_radius * 2  # diameter
        robot_marker.scale.z = self.robot_height
        
        # Color (blue with transparency, red if victory)
        if self.victory_state:
            robot_marker.color.r = 0.8
            robot_marker.color.g = 0.2
            robot_marker.color.b = 0.2
            robot_marker.color.a = 0.9
        else:
            robot_marker.color.r = 0.2
            robot_marker.color.g = 0.4
            robot_marker.color.b = 0.8
            robot_marker.color.a = 0.8
        
        robot_marker.lifetime.sec = 0
        robot_marker.lifetime.nanosec = 0  # Persistent
        
        self.marker_publisher.publish(robot_marker)
        
        # Direction indicator
        direction_marker = Marker()
        direction_marker.header.frame_id = "base_link"
        direction_marker.header.stamp = robot_marker.header.stamp
        direction_marker.ns = "chassis_robot"
        direction_marker.id = 1
        direction_marker.type = Marker.CYLINDER
        direction_marker.action = Marker.ADD
        
        # Position (front of the robot)
        direction_marker.pose.position.x = self.robot_radius - 0.01
        direction_marker.pose.position.y = 0.0
        direction_marker.pose.position.z = self.robot_height + 0.01
        
        # Orientation
        direction_marker.pose.orientation.x = 0.0
        direction_marker.pose.orientation.y = 0.0
        direction_marker.pose.orientation.z = 0.0
        direction_marker.pose.orientation.w = 1.0
        
        # Scale (small cylinder)
        direction_marker.scale.x = 0.02  # diameter
        direction_marker.scale.y = 0.02  # diameter
        direction_marker.scale.z = 0.02  # height
        
        # Color (red)
        direction_marker.color.r = 0.8
        direction_marker.color.g = 0.2
        direction_marker.color.b = 0.2
        direction_marker.color.a = 1.0
        
        direction_marker.lifetime.sec = 0
        direction_marker.lifetime.nanosec = 0
        
        self.marker_publisher.publish(direction_marker)
        
        # Velocity vector if robot is moving
        if self.current_velocity and (abs(self.current_velocity.linear.x) > 0.01 or 
                                     abs(self.current_velocity.linear.y) > 0.01):
            velocity_marker = Marker()
            velocity_marker.header.frame_id = "base_link"
            velocity_marker.header.stamp = robot_marker.header.stamp
            velocity_marker.ns = "velocity"
            velocity_marker.id = 2
            velocity_marker.type = Marker.ARROW
            velocity_marker.action = Marker.ADD
            
            # Start point (center of robot)
            velocity_marker.pose.position.x = 0.0
            velocity_marker.pose.position.y = 0.0
            velocity_marker.pose.position.z = self.robot_height + 0.05
            
            # Direction of velocity vector
            vx = self.current_velocity.linear.x
            vy = self.current_velocity.linear.y
            vel_magnitude = math.sqrt(vx*vx + vy*vy)
            
            if vel_magnitude > 0.01:
                vel_yaw = math.atan2(vy, vx)
                vel_quat = tf_transformations.quaternion_from_euler(0, 0, vel_yaw)
                velocity_marker.pose.orientation.x = vel_quat[0]
                velocity_marker.pose.orientation.y = vel_quat[1]
                velocity_marker.pose.orientation.z = vel_quat[2]
                velocity_marker.pose.orientation.w = vel_quat[3]
            
            # Scale (proportional to velocity)
            arrow_length = min(vel_magnitude * 0.5, 0.3)  # Max 30cm arrow
            velocity_marker.scale.x = arrow_length
            velocity_marker.scale.y = 0.02
            velocity_marker.scale.z = 0.02
            
            # Color (green)
            velocity_marker.color.r = 0.2
            velocity_marker.color.g = 0.8
            velocity_marker.color.b = 0.2
            velocity_marker.color.a = 0.8
            
            # Lifetime
            velocity_marker.lifetime.sec = 0
            velocity_marker.lifetime.nanosec = 200000000  # 0.2 seconds
            
            self.marker_publisher.publish(velocity_marker)

def main(args=None):
    rclpy.init(args=args)
    
    node = ChassisVisualization()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Chassis visualization stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
