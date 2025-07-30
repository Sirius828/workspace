#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class SimpleNavigator : public rclcpp::Node
{
public:
    SimpleNavigator() : Node("simple_navigator")
    {
        // 声明参数
        this->declare_parameter("linear_kp", 1.0);
        this->declare_parameter("angular_kp", 2.0);
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 1.0);
        this->declare_parameter("position_tolerance", 0.1);
        this->declare_parameter("angle_tolerance", 0.1);
        this->declare_parameter("approach_distance", 0.5);
        
        // 获取参数
        linear_kp_ = this->get_parameter("linear_kp").as_double();
        angular_kp_ = this->get_parameter("angular_kp").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        position_tolerance_ = this->get_parameter("position_tolerance").as_double();
        angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
        approach_distance_ = this->get_parameter("approach_distance").as_double();
        
        // 订阅里程计话题
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diff_drive_controller/odom", 10,
            std::bind(&SimpleNavigator::odom_callback, this, std::placeholders::_1));
            
        // 订阅目标位置话题
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&SimpleNavigator::goal_callback, this, std::placeholders::_1));
            
        // 发布速度命令
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diff_drive_controller/cmd_vel_unstamped", 10);
            
        // 控制循环定时器 (100Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SimpleNavigator::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "简单导航器已启动");
        RCLCPP_INFO(this->get_logger(), "订阅话题: /diff_drive_controller/odom, /goal_pose");
        RCLCPP_INFO(this->get_logger(), "发布话题: /diff_drive_controller/cmd_vel_unstamped");
        RCLCPP_INFO(this->get_logger(), "控制参数: linear_kp=%.2f, angular_kp=%.2f", linear_kp_, angular_kp_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        has_odom_ = true;
        
        if (!first_odom_received_) {
            first_odom_received_ = true;
            RCLCPP_INFO(this->get_logger(), "收到第一个里程计数据");
        }
    }
    
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_pose_ = msg->pose;
        has_goal_ = true;
        goal_reached_ = false;
        
        double goal_yaw = get_yaw(goal_pose_.orientation);
        RCLCPP_INFO(this->get_logger(), 
                   "收到新目标位置: x=%.2f, y=%.2f, yaw=%.2f° (%.3f rad)", 
                   goal_pose_.position.x, goal_pose_.position.y, 
                   goal_yaw * 180.0 / M_PI, goal_yaw);
    }
    
    void control_loop()
    {
        if (!has_odom_ || !has_goal_ || goal_reached_) {
            // 如果没有里程计数据、目标或已到达，停止机器人
            if (has_odom_ && !has_goal_) {
                geometry_msgs::msg::Twist stop_cmd;
                cmd_pub_->publish(stop_cmd);
            }
            return;
        }
        
        // 计算位置误差
        double dx = goal_pose_.position.x - current_pose_.position.x;
        double dy = goal_pose_.position.y - current_pose_.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        // 计算角度
        double target_yaw = atan2(dy, dx);
        double current_yaw = get_yaw(current_pose_.orientation);
        double goal_yaw = get_yaw(goal_pose_.orientation);
        
        geometry_msgs::msg::Twist cmd;
        
        // 三阶段控制策略
        if (distance > position_tolerance_) {
            // 阶段1&2: 导航到目标位置
            double yaw_error = normalize_angle(target_yaw - current_yaw);
            
            if (distance > approach_distance_ && fabs(yaw_error) > angle_tolerance_) {
                // 距离较远且朝向偏差大：先原地转向
                cmd.angular.z = compute_angular_velocity(yaw_error);
                RCLCPP_DEBUG(this->get_logger(), "阶段1: 原地转向, yaw_error=%.2f°", yaw_error * 180.0 / M_PI);
            } else {
                // 距离较近或朝向正确：前进并微调
                double speed_factor = std::min(1.0, distance / approach_distance_);
                cmd.linear.x = linear_kp_ * distance * speed_factor;
                cmd.linear.x = std::max(0.05, std::min(max_linear_vel_, cmd.linear.x));
                
                // 边走边微调朝向
                cmd.angular.z = compute_angular_velocity(yaw_error);
                
                RCLCPP_DEBUG(this->get_logger(), "阶段2: 导航前进, distance=%.2f, speed=%.2f", distance, cmd.linear.x);
            }
        } else {
            // 阶段3: 到达位置，调整最终朝向
            double final_yaw_error = normalize_angle(goal_yaw - current_yaw);
            
            if (fabs(final_yaw_error) > angle_tolerance_) {
                cmd.angular.z = compute_angular_velocity(final_yaw_error);
                RCLCPP_DEBUG(this->get_logger(), "阶段3: 最终朝向调整, yaw_error=%.2f°", final_yaw_error * 180.0 / M_PI);
            } else {
                // 完全到达目标
                goal_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "✅ 目标已到达！位置误差: %.3fm, 角度误差: %.2f°", 
                           distance, fabs(final_yaw_error) * 180.0 / M_PI);
            }
        }
        
        // 发布控制命令
        cmd_pub_->publish(cmd);
        
        // 定期输出状态信息
        if (++debug_counter_ >= 40) {  // 每2秒输出一次 (40 * 50ms)
            debug_counter_ = 0;
            RCLCPP_INFO(this->get_logger(), 
                       "状态: 距离=%.2fm, 当前位置=(%.2f,%.2f), 目标位置=(%.2f,%.2f)", 
                       distance, current_pose_.position.x, current_pose_.position.y,
                       goal_pose_.position.x, goal_pose_.position.y);
        }
    }
    
    double compute_angular_velocity(double yaw_error)
    {
        double angular_vel = angular_kp_ * yaw_error;
        return std::max(-max_angular_vel_, std::min(max_angular_vel_, angular_vel));
    }
    
    double get_yaw(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    // ROS组件
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 状态变量
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Pose goal_pose_;
    bool has_odom_ = false;
    bool has_goal_ = false;
    bool goal_reached_ = false;
    bool first_odom_received_ = false;
    int debug_counter_ = 0;
    
    // 控制参数
    double linear_kp_;
    double angular_kp_;
    double max_linear_vel_;
    double max_angular_vel_;
    double position_tolerance_;
    double angle_tolerance_;
    double approach_distance_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleNavigator>());
    rclcpp::shutdown();
    return 0;
}
