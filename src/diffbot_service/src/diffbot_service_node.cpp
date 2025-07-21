#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

enum class State {
    IDLE,
    MOVE_TO_FIRST_POSITION,
    WAIT_FOR_NUMBER,
    MOVE_TO_SECOND_POSITION,
    ROTATE_CLOCKWISE,
    MOVE_TO_CIRCLE_START,
    ROTATE_COUNTERCLOCKWISE,
    EXECUTE_CIRCLE_TRAJECTORY,
    COMPLETED
};

struct TrajectoryPoint {
    double x, y, yaw;
};

class DiffBotService : public rclcpp::Node
{
public:
    DiffBotService() : Node("diffbot_service_node")
    {
        // 声明参数
        this->declare_parameter("odom_topic", "/diff_drive_controller/odom");
        this->declare_parameter("first_move_x", 0.05);
        this->declare_parameter("second_move_x", 0.2);
        this->declare_parameter("circle_interpolation_points", 50);
        this->declare_parameter("position_tolerance", 0.05);
        this->declare_parameter("angle_tolerance", 0.1);
        this->declare_parameter("linear_kp", 1.0);
        this->declare_parameter("angular_kp", 1.0);
        this->declare_parameter("max_linear_vel", 0.3);
        this->declare_parameter("max_angular_vel", 0.8);
        
        // 读取参数
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        first_move_x_ = this->get_parameter("first_move_x").as_double();
        second_move_x_ = this->get_parameter("second_move_x").as_double();
        circle_points_ = this->get_parameter("circle_interpolation_points").as_int();
        position_tolerance_ = this->get_parameter("position_tolerance").as_double();
        angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
        linear_kp_ = this->get_parameter("linear_kp").as_double();
        angular_kp_ = this->get_parameter("angular_kp").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        
        // 初始化订阅者
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&DiffBotService::odom_callback, this, std::placeholders::_1));
            
        number_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/number", 10,
            std::bind(&DiffBotService::number_callback, this, std::placeholders::_1));
        
        // 初始化发布者
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        
        // 初始化服务
        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start",
            std::bind(&DiffBotService::start_callback, this, std::placeholders::_1, std::placeholders::_2));
            
        // 控制循环定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DiffBotService::control_loop, this));
        
        // 初始化状态
        current_state_ = State::IDLE;
        has_odom_ = false;
        has_number_ = false;
        radius_cm_ = 0.0;
        current_trajectory_point_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "DiffBot Service 节点已启动");
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s, /number", odom_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "发布话题: /goal_pose");
        RCLCPP_INFO(this->get_logger(), "服务: /start");
        RCLCPP_INFO(this->get_logger(), "配置: 第一次移动x=%.3f, 第二次移动x=%.3f, 圆形插值点=%d", 
                   first_move_x_, second_move_x_, circle_points_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        has_odom_ = true;
        
        if (!first_odom_received_) {
            first_odom_received_ = true;
            RCLCPP_INFO(this->get_logger(), "收到第一个里程计数据: (%.3f, %.3f, %.1f°)",
                       current_pose_.position.x, current_pose_.position.y, 
                       get_yaw(current_pose_.orientation) * 180.0 / M_PI);
        }
    }
    
    void number_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (current_state_ == State::WAIT_FOR_NUMBER && !has_number_) {
            radius_cm_ = static_cast<double>(msg->data);
            has_number_ = true;
            RCLCPP_INFO(this->get_logger(), "收到半径数据: %.1f cm (%.3f m)", 
                       radius_cm_, radius_cm_ / 100.0);
        }
    }
    
    void start_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;  // 避免未使用变量警告
        
        if (!has_odom_) {
            response->success = false;
            response->message = "没有收到里程计数据，无法启动";
            RCLCPP_ERROR(this->get_logger(), "启动失败: %s", response->message.c_str());
            return;
        }
        
        // 记录起始位置
        start_pose_ = current_pose_;
        current_state_ = State::MOVE_TO_FIRST_POSITION;
        has_number_ = false;
        radius_cm_ = 0.0;
        current_trajectory_point_ = 0;
        circle_trajectory_.clear();
        
        response->success = true;
        response->message = "任务已启动，开始执行运动序列";
        
        RCLCPP_INFO(this->get_logger(), "启动服务被调用");
        RCLCPP_INFO(this->get_logger(), "起始位置: (%.3f, %.3f, %.1f°)", 
                   start_pose_.position.x, start_pose_.position.y,
                   get_yaw(start_pose_.orientation) * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "开始执行任务序列...");
    }
    
    void control_loop()
    {
        if (!has_odom_) return;
        
        switch (current_state_) {
            case State::IDLE:
                // 等待服务调用
                break;
                
            case State::MOVE_TO_FIRST_POSITION:
                handle_move_to_first_position();
                break;
                
            case State::WAIT_FOR_NUMBER:
                handle_wait_for_number();
                break;
                
            case State::MOVE_TO_SECOND_POSITION:
                handle_move_to_second_position();
                break;
                
            case State::ROTATE_CLOCKWISE:
                handle_rotate_clockwise();
                break;
                
            case State::MOVE_TO_CIRCLE_START:
                handle_move_to_circle_start();
                break;
                
            case State::ROTATE_COUNTERCLOCKWISE:
                handle_rotate_counterclockwise();
                break;
                
            case State::EXECUTE_CIRCLE_TRAJECTORY:
                handle_execute_circle_trajectory();
                break;
                
            case State::COMPLETED:
                // 任务完成
                break;
        }
    }
    
    void handle_move_to_first_position()
    {
        // 移动到 (first_move_x, 0) 相对于起始位置
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "odom";
        goal.header.stamp = this->get_clock()->now();
        goal.pose.position.x = start_pose_.position.x + first_move_x_;
        goal.pose.position.y = start_pose_.position.y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation = start_pose_.orientation;  // 保持原来的朝向
        
        goal_pub_->publish(goal);
        
        // 检查是否到达
        double dx = goal.pose.position.x - current_pose_.position.x;
        double dy = goal.pose.position.y - current_pose_.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        if (distance < position_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "到达第一个位置: (%.3f, %.3f)", 
                       current_pose_.position.x, current_pose_.position.y);
            current_state_ = State::WAIT_FOR_NUMBER;
        }
    }
    
    void handle_wait_for_number()
    {
        if (has_number_) {
            RCLCPP_INFO(this->get_logger(), "收到半径数据，继续下一步");
            current_state_ = State::MOVE_TO_SECOND_POSITION;
        }
        // 在等待期间停止发布目标
    }
    
    void handle_move_to_second_position()
    {
        // 移动到 (second_move_x, 0) 相对于起始位置
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "odom";
        goal.header.stamp = this->get_clock()->now();
        goal.pose.position.x = start_pose_.position.x + second_move_x_;
        goal.pose.position.y = start_pose_.position.y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation = start_pose_.orientation;
        
        goal_pub_->publish(goal);
        
        // 检查是否到达
        double dx = goal.pose.position.x - current_pose_.position.x;
        double dy = goal.pose.position.y - current_pose_.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        if (distance < position_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "到达第二个位置: (%.3f, %.3f)", 
                       current_pose_.position.x, current_pose_.position.y);
            current_state_ = State::ROTATE_CLOCKWISE;
        }
    }
    
    void handle_rotate_clockwise()
    {
        // 顺时针旋转90度
        double current_yaw = get_yaw(current_pose_.orientation);
        double target_yaw = get_yaw(start_pose_.orientation) - M_PI/2;  // 顺时针90度
        
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "odom";
        goal.header.stamp = this->get_clock()->now();
        goal.pose.position = current_pose_.position;  // 保持当前位置
        goal.pose.orientation = create_quaternion_from_yaw(target_yaw);
        
        goal_pub_->publish(goal);
        
        // 检查是否到达目标朝向
        double yaw_error = normalize_angle(target_yaw - current_yaw);
        if (fabs(yaw_error) < angle_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "完成顺时针旋转90度");
            current_state_ = State::MOVE_TO_CIRCLE_START;
        }
    }
    
    void handle_move_to_circle_start()
    {
        // 移动到 (current_x, current_y + radius)
        double radius_m = radius_cm_ / 100.0;
        
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "odom";
        goal.header.stamp = this->get_clock()->now();
        goal.pose.position.x = current_pose_.position.x;
        goal.pose.position.y = current_pose_.position.y + radius_m;
        goal.pose.position.z = 0.0;
        goal.pose.orientation = current_pose_.orientation;  // 保持当前朝向
        
        goal_pub_->publish(goal);
        
        // 检查是否到达
        double dx = goal.pose.position.x - current_pose_.position.x;
        double dy = goal.pose.position.y - current_pose_.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        if (distance < position_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "到达圆形起始位置: (%.3f, %.3f)", 
                       current_pose_.position.x, current_pose_.position.y);
            circle_center_x_ = current_pose_.position.x;
            circle_center_y_ = current_pose_.position.y - radius_m;  // 圆心在下方
            current_state_ = State::ROTATE_COUNTERCLOCKWISE;
        }
    }
    
    void handle_rotate_counterclockwise()
    {
        // 逆时针旋转90度
        double current_yaw = get_yaw(current_pose_.orientation);
        double target_yaw = get_yaw(start_pose_.orientation);  // 回到原始朝向
        
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "odom";
        goal.header.stamp = this->get_clock()->now();
        goal.pose.position = current_pose_.position;  // 保持当前位置
        goal.pose.orientation = create_quaternion_from_yaw(target_yaw);
        
        goal_pub_->publish(goal);
        
        // 检查是否到达目标朝向
        double yaw_error = normalize_angle(target_yaw - current_yaw);
        if (fabs(yaw_error) < angle_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "完成逆时针旋转90度，开始生成圆形轨迹");
            generate_circle_trajectory();
            current_trajectory_point_ = 0;
            current_state_ = State::EXECUTE_CIRCLE_TRAJECTORY;
        }
    }
    
    void handle_execute_circle_trajectory()
    {
        if (current_trajectory_point_ >= circle_trajectory_.size()) {
            RCLCPP_INFO(this->get_logger(), "圆形轨迹执行完成！");
            current_state_ = State::COMPLETED;
            return;
        }
        
        // 发布当前轨迹点
        const auto& point = circle_trajectory_[current_trajectory_point_];
        
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "odom";
        goal.header.stamp = this->get_clock()->now();
        goal.pose.position.x = point.x;
        goal.pose.position.y = point.y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation = create_quaternion_from_yaw(point.yaw);
        
        goal_pub_->publish(goal);
        
        // 检查是否到达当前点
        double dx = point.x - current_pose_.position.x;
        double dy = point.y - current_pose_.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        if (distance < position_tolerance_) {
            if (current_trajectory_point_ % 10 == 0) {  // 每10个点打印一次
                RCLCPP_INFO(this->get_logger(), "圆形轨迹点 %zu/%zu: (%.3f, %.3f, %.1f°)",
                           current_trajectory_point_ + 1, circle_trajectory_.size(),
                           point.x, point.y, point.yaw * 180.0 / M_PI);
            }
            current_trajectory_point_++;
        }
    }
    
    void generate_circle_trajectory()
    {
        circle_trajectory_.clear();
        double radius_m = radius_cm_ / 100.0;
        
        // 生成圆形轨迹点
        for (int i = 0; i < circle_points_; ++i) {
            double angle = 2.0 * M_PI * i / circle_points_;
            
            TrajectoryPoint point;
            point.x = circle_center_x_ + radius_m * cos(angle);
            point.y = circle_center_y_ + radius_m * sin(angle);
            
            // 计算切线方向作为朝向
            point.yaw = angle + M_PI/2;  // 切线方向
            point.yaw = normalize_angle(point.yaw);
            
            circle_trajectory_.push_back(point);
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "生成圆形轨迹：半径=%.1fcm, 圆心=(%.3f, %.3f), 轨迹点数=%zu",
                   radius_cm_, circle_center_x_, circle_center_y_, circle_trajectory_.size());
    }
    
    double get_yaw(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    geometry_msgs::msg::Quaternion create_quaternion_from_yaw(double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    }
    
    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    // ROS组件
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr number_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 状态变量
    State current_state_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Pose start_pose_;
    bool has_odom_;
    bool has_number_;
    bool first_odom_received_ = false;
    double radius_cm_;
    
    // 圆形轨迹
    std::vector<TrajectoryPoint> circle_trajectory_;
    size_t current_trajectory_point_;
    double circle_center_x_, circle_center_y_;
    
    // 配置参数
    std::string odom_topic_;
    double first_move_x_;
    double second_move_x_;
    int circle_points_;
    double position_tolerance_;
    double angle_tolerance_;
    double linear_kp_;
    double angular_kp_;
    double max_linear_vel_;
    double max_angular_vel_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffBotService>());
    rclcpp::shutdown();
    return 0;
}
