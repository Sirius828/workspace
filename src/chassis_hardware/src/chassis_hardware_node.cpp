#include "chassis_hardware/chassis_hardware_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace chassis_hardware
{

ChassisHardwareNode::ChassisHardwareNode() 
    : Node("chassis_hardware_node"),
      running_(true),
      current_vx_(0.0), current_vy_(0.0), current_vw_(0.0),
      current_gimbal_yaw_(0.0), current_gimbal_pitch_(0.0), current_victory_state_(false),
      victory_topic_active_(false), victory_timeout_duration_(1.0),
      accumulated_x_(0.0), accumulated_y_(0.0), accumulated_yaw_(0.0)
{
    // Declare parameters
    this->declare_parameter("serial_port", "/dev/ttyCH341USB0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("base_frame_id", "base_link");
    this->declare_parameter("odom_frame_id", "odom");
    this->declare_parameter("victory_timeout", 1.0);  // 胜利消息超时时间（秒）
    
    // Get parameters
    serial_port_name_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    victory_timeout_duration_ = this->get_parameter("victory_timeout").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Initializing chassis hardware node");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s, Baud rate: %d", 
                serial_port_name_.c_str(), baud_rate_);
    RCLCPP_INFO(this->get_logger(), "Victory timeout: %.1f seconds", victory_timeout_duration_);
    
    // Initialize publishers
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/chassis/odom", 10);
    
    // Initialize subscribers
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&ChassisHardwareNode::cmdVelCallback, this, std::placeholders::_1)
    );
    
    cmd_gimbal_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_gimbal", 10,
        std::bind(&ChassisHardwareNode::cmdGimbalCallback, this, std::placeholders::_1)
    );
    
    victory_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/victory", 10,
        std::bind(&ChassisHardwareNode::victoryCallback, this, std::placeholders::_1)
    );
    
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Initialize time
    last_time_ = this->get_clock()->now();
    last_victory_msg_time_ = this->get_clock()->now();  // 初始化胜利消息时间
    
    // Initialize serial communication
    initializeSerial();
    
    // Start serial reading thread
    serial_thread_ = std::thread(&ChassisHardwareNode::serialReadThread, this);
    
    RCLCPP_INFO(this->get_logger(), "Chassis hardware node initialized successfully");
}

ChassisHardwareNode::~ChassisHardwareNode()
{
    running_ = false;
    
    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
    
    if (serial_port_ && serial_port_->isOpen()) {
        serial_port_->close();
    }
    
    RCLCPP_INFO(this->get_logger(), "Chassis hardware node destroyed");
}

void ChassisHardwareNode::initializeSerial()
{
    try {
        serial_port_ = std::make_unique<serial::Serial>(
            serial_port_name_, 
            baud_rate_, 
            serial::Timeout::simpleTimeout(1000)
        );
        
        if (serial_port_->isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial port initialization error: %s", e.what());
    }
}

void ChassisHardwareNode::serialReadThread()
{
    std::string buffer;
    
    while (running_ && rclcpp::ok()) {
        try {
            if (serial_port_ && serial_port_->isOpen()) {
                if (serial_port_->available()) {
                    std::string data = serial_port_->read(serial_port_->available());
                    buffer += data;
                    
                    // 输出原始接收数据用于调试
                    if (!data.empty()) {
                        RCLCPP_DEBUG(this->get_logger(), "Raw serial data received: '%s' (hex: ", data.c_str());
                        for (char c : data) {
                            RCLCPP_DEBUG(this->get_logger(), "%02X ", static_cast<unsigned char>(c));
                        }
                        RCLCPP_DEBUG(this->get_logger(), ")");
                    }
                    
                    // Process complete lines ending with \r\n
                    size_t pos = 0;
                    while ((pos = buffer.find("\r\n")) != std::string::npos) {
                        std::string line = buffer.substr(0, pos);
                        buffer.erase(0, pos + 2);
                        
                        if (!line.empty()) {
                            RCLCPP_DEBUG(this->get_logger(), "Processing line: '%s'", line.c_str());
                            parseOdometryData(line);
                        }
                    }
                    
                    // 如果缓冲区过长，清空它以防止内存问题
                    if (buffer.length() > 1000) {
                        RCLCPP_WARN(this->get_logger(), "Serial buffer too long, clearing. Buffer was: '%s'", buffer.c_str());
                        buffer.clear();
                    }
                }
            }
            
            // Small delay to prevent excessive CPU usage
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void ChassisHardwareNode::parseOdometryData(const std::string& data)
{
    try {
        // 输出接收到的原始数据用于调试
        RCLCPP_DEBUG(this->get_logger(), "Received raw data: '%s' (length: %zu)", data.c_str(), data.length());
        
        // 检查数据是否以"ODOM,"开头
        if (data.length() < 5 || data.substr(0, 5) != "ODOM,") {
            RCLCPP_WARN(this->get_logger(), "Data does not start with 'ODOM,': '%s'", data.c_str());
            return;
        }
        
        // 去掉"ODOM,"前缀，获取实际的数据部分
        std::string odom_data = data.substr(5);
        RCLCPP_DEBUG(this->get_logger(), "Odometry data after removing prefix: '%s'", odom_data.c_str());
        
        std::istringstream iss(odom_data);
        std::string token;
        std::vector<double> values;
        std::vector<std::string> raw_tokens;
        
        // Parse comma-separated values
        while (std::getline(iss, token, ',')) {
            // 去除token前后的空白字符
            token.erase(0, token.find_first_not_of(" \t\n\r\f\v"));
            token.erase(token.find_last_not_of(" \t\n\r\f\v") + 1);
            
            raw_tokens.push_back(token);
            
            if (!token.empty()) {
                try {
                    double value = std::stod(token);
                    values.push_back(value);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse token '%s': %s", token.c_str(), e.what());
                    return;
                }
            }
        }
        
        // 输出解析后的token用于调试
        std::ostringstream debug_oss;
        for (size_t i = 0; i < raw_tokens.size(); ++i) {
            debug_oss << "[" << i << "]:'" << raw_tokens[i] << "'";
            if (i < raw_tokens.size() - 1) debug_oss << ", ";
        }
        RCLCPP_DEBUG(this->get_logger(), "Parsed tokens: %s", debug_oss.str().c_str());
        
        // Expected format: "ODOM,x累计位移,y累计位移,累计偏航角,x速度,y速度,角速度\r\n"
        // 所有单位都是ROS标准单位: 位置(m), 角度(rad), 速度(m/s), 角速度(rad/s)
        if (values.size() == 6) {
            double x_position = values[0];     // 累计x位移 (m)
            double y_position = values[1];     // 累计y位移 (m)
            double yaw_total = values[2];      // 累计偏航角 (rad)
            double vx = values[3];             // x方向速度 (m/s)
            double vy = values[4];             // y方向速度 (m/s)
            double vw = values[5];             // 角速度 (rad/s)
            
            RCLCPP_DEBUG(this->get_logger(), "Parsed odometry: x_pos=%.3f m, y_pos=%.3f m, yaw=%.3f rad, vx=%.3f m/s, vy=%.3f m/s, vw=%.3f rad/s", 
                        x_position, y_position, yaw_total, vx, vy, vw);
            
            publishOdometry(x_position, y_position, yaw_total, vx, vy, vw);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid odometry data format: expected 6 values, got %zu. Raw data: '%s'", 
                       values.size(), data.c_str());
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing odometry data: %s. Raw data: '%s'", e.what(), data.c_str());
    }
}

void ChassisHardwareNode::publishOdometry(double x_offset, double y_offset, double yaw_total,
                                         double vx, double vy, double vw)
{
    auto current_time = this->get_clock()->now();
    
    // Create odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    
    // 注意：接收到的数据已经是ROS标准单位(m, m/s, rad, rad/s)
    // x_offset, y_offset 是累计位置偏移，不是增量
    // yaw_total 是累计偏航角
    // vx, vy, vw 是当前速度
    
    // 直接使用接收到的累计位置
    accumulated_x_ = x_offset;
    accumulated_y_ = y_offset;
    accumulated_yaw_ = yaw_total;
    
    odom_msg.pose.pose.position.x = accumulated_x_;
    odom_msg.pose.pose.position.y = accumulated_y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, accumulated_yaw_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    
    // Set velocity
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = vw;
    
    // Set covariance (can be tuned based on sensor accuracy)
    std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
    std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
    
    // Diagonal covariance values
    odom_msg.pose.covariance[0] = 0.1;   // x
    odom_msg.pose.covariance[7] = 0.1;   // y
    odom_msg.pose.covariance[35] = 0.1;  // yaw
    odom_msg.twist.covariance[0] = 0.1;  // vx
    odom_msg.twist.covariance[7] = 0.1;  // vy
    odom_msg.twist.covariance[35] = 0.1; // vw
    
    // Publish odometry
    odom_publisher_->publish(odom_msg);
    
    // Broadcast transform
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_id_;
    odom_trans.child_frame_id = base_frame_id_;
    
    odom_trans.transform.translation.x = accumulated_x_;
    odom_trans.transform.translation.y = accumulated_y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf2::toMsg(q);
    
    tf_broadcaster_->sendTransform(odom_trans);
    
    last_time_ = current_time;
}

void ChassisHardwareNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    current_vx_ = msg->linear.x;
    current_vy_ = msg->linear.y;
    current_vw_ = msg->angular.z;
    
    // Send command to serial port
    sendCommandToSerial(current_vx_, current_vy_, current_vw_, 
                       current_gimbal_yaw_, current_gimbal_pitch_, current_victory_state_);
}

void ChassisHardwareNode::cmdGimbalCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Assuming gimbal angles are sent as angular x (yaw) and y (pitch)
    current_gimbal_yaw_ = -msg->angular.x;
    current_gimbal_pitch_ = msg->angular.y;
    
    // Send command to serial port
    sendCommandToSerial(current_vx_, current_vy_, current_vw_, 
                       current_gimbal_yaw_, current_gimbal_pitch_, current_victory_state_);
}

void ChassisHardwareNode::victoryCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool previous_victory_state = current_victory_state_;
    current_victory_state_ = msg->data;
    
    // 记录收到victory消息的时间
    last_victory_msg_time_ = this->get_clock()->now();
    victory_topic_active_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Victory message received: %s -> %s", 
                previous_victory_state ? "true" : "false",
                current_victory_state_ ? "true" : "false");
    
    // Send updated command to serial port
    sendCommandToSerial(current_vx_, current_vy_, current_vw_, 
                       current_gimbal_yaw_, current_gimbal_pitch_, current_victory_state_);
}

void ChassisHardwareNode::sendCommandToSerial(double vx, double vy, double vw, 
                                             double gimbal_yaw, double gimbal_pitch, bool victory_state)
{
    try {
        if (serial_port_ && serial_port_->isOpen()) {
            // Convert gimbal angles from radians to degrees
            double gimbal_yaw_deg = gimbal_yaw * 180.0 / M_PI;
            double gimbal_pitch_deg = gimbal_pitch * 180.0 / M_PI;
            
            // Convert linear velocities from m/s to mm/s for lower computer
            double vx_mm_s = vx * 1000.0;  // m/s -> mm/s
            double vy_mm_s = -vy * 1000.0;  // m/s -> mm/s (反向y方向)
            // Angular velocity remains in rad/s
            
            // 检查victory话题是否超时
            bool effective_victory_state = false;
            if (victory_topic_active_) {
                auto current_time = this->get_clock()->now();
                double time_since_last_victory = (current_time - last_victory_msg_time_).seconds();
                
                if (time_since_last_victory <= victory_timeout_duration_) {
                    // 在超时时间内且收到了victory=true，才发送1
                    effective_victory_state = victory_state;
                    RCLCPP_DEBUG(this->get_logger(), "Victory active: state=%s, time_since_last=%.2fs", 
                                victory_state ? "true" : "false", time_since_last_victory);
                } else {
                    // 超时了，发送0
                    effective_victory_state = false;
                    victory_topic_active_ = false;  // 标记为不活跃
                    RCLCPP_DEBUG(this->get_logger(), "Victory timeout: %.2fs > %.2fs, sending 0", 
                                time_since_last_victory, victory_timeout_duration_);
                }
            } else {
                // 没有收到过victory消息，发送0
                effective_victory_state = false;
                RCLCPP_DEBUG(this->get_logger(), "No victory message received, sending 0");
            }
            
            // Format: "x速度(mm/s),y速度(mm/s),角速度(rad/s),yaw角度(度),pitch角度(度),victory状态\r\n"
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6);
            oss << vx_mm_s << "," << vy_mm_s << "," << vw << "," 
                << gimbal_yaw_deg << "," << gimbal_pitch_deg << "," << (effective_victory_state ? 1 : 0) << "\r\n";
            
            std::string command = oss.str();
            
            // 输出原始串口数据，包括实际发送的victory状态
            if (effective_victory_state != victory_state) {
                RCLCPP_INFO(this->get_logger(), "Sending raw serial data: '%s' (victory overridden: %s -> %s)", 
                           command.c_str(), victory_state ? "true" : "false", 
                           effective_victory_state ? "true" : "false");
            } else {
                RCLCPP_INFO(this->get_logger(), "Sending raw serial data: '%s'", command.c_str());
            }
            
            // 输出十六进制调试信息
            std::ostringstream hex_oss;
            for (char c : command) {
                hex_oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned char>(c) << " ";
            }
            RCLCPP_DEBUG(this->get_logger(), "Command hex: %s", hex_oss.str().c_str());
            
            serial_port_->write(command);
            
        } else {
            RCLCPP_WARN(this->get_logger(), "Serial port not open, cannot send command");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error sending command to serial: %s", e.what());
    }
}

} // namespace chassis_hardware
