#ifndef CHASSIS_HARDWARE_NODE_HPP
#define CHASSIS_HARDWARE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <serial/serial.h>
#include <string>
#include <memory>
#include <thread>
#include <atomic>

namespace chassis_hardware
{

class ChassisHardwareNode : public rclcpp::Node
{
public:
    ChassisHardwareNode();
    ~ChassisHardwareNode();

private:
    // Serial communication
    void initializeSerial();
    void serialReadThread();
    void parseOdometryData(const std::string& data);
    void sendCommandToSerial(double vx, double vy, double vw, double gimbal_yaw, double gimbal_pitch, bool victory_state);
    
    // ROS callbacks
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void cmdGimbalCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void victoryCallback(const std_msgs::msg::Bool::SharedPtr msg);
    
    // Odometry publishing
    void publishOdometry(double x_offset, double y_offset, double yaw_total, 
                        double vx, double vy, double vw);
    
    // Member variables
    std::unique_ptr<serial::Serial> serial_port_;
    std::thread serial_thread_;
    std::atomic<bool> running_;
    
    // ROS publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_gimbal_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr victory_subscriber_;
    
    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Current command values
    double current_vx_, current_vy_, current_vw_;
    double current_gimbal_yaw_, current_gimbal_pitch_;
    bool current_victory_state_;
    
    // Victory topic monitoring
    rclcpp::Time last_victory_msg_time_;
    bool victory_topic_active_;
    double victory_timeout_duration_;  // 胜利消息超时时间 (秒)
    
    // Odometry data
    double accumulated_x_, accumulated_y_, accumulated_yaw_;
    rclcpp::Time last_time_;
    
    // Parameters
    std::string serial_port_name_;
    int baud_rate_;
    std::string base_frame_id_;
    std::string odom_frame_id_;
};

} // namespace chassis_hardware

#endif // CHASSIS_HARDWARE_NODE_HPP
