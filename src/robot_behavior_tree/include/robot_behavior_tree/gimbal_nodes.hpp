#ifndef ROBOT_BEHAVIOR_TREE_GIMBAL_NODES_HPP
#define ROBOT_BEHAVIOR_TREE_GIMBAL_NODES_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

namespace robot_behavior_tree {

/**
 * @brief 控制云台转向指定角度的行为树节点
 */
class GimbalRotateTo : public BT::StatefulActionNode {
public:
    GimbalRotateTo(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("yaw_angle", "Target yaw angle in radians"),
            BT::InputPort<double>("pitch_angle", "Target pitch angle in radians"),
            BT::InputPort<double>("timeout", 5.0, "Timeout in seconds")
        };
    }
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gimbal_cmd_pub_;
    
    double target_yaw_;
    double target_pitch_;
    double timeout_;
    std::chrono::steady_clock::time_point start_time_;
    
    void setupROS();
};

/**
 * @brief 启用云台目标追踪的行为树节点
 */
class EnableGimbalTracking : public BT::SyncActionNode {
public:
    EnableGimbalTracking(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<bool>("enable", true, "Enable or disable tracking")
        };
    }
    
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tracking_enable_pub_;
    
    void setupROS();
};

/**
 * @brief 云台回到中央位置的行为树节点
 */
class GimbalCenter : public BT::StatefulActionNode {
public:
    GimbalCenter(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("timeout", 3.0, "Timeout in seconds")
        };
    }
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gimbal_cmd_pub_;
    
    double timeout_;
    std::chrono::steady_clock::time_point start_time_;
    
    void setupROS();
};

/**
 * @brief 检查云台是否正在追踪目标的条件节点
 */
class IsGimbalTracking : public BT::ConditionNode {
public:
    IsGimbalTracking(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("timeout", 2.0, "Tracking timeout in seconds")
        };
    }
    
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    
    std::chrono::steady_clock::time_point last_target_time_;
    bool target_received_ = false;
    double timeout_;
    
    void setupROS();
    void targetCallback(const geometry_msgs::msg::Point::SharedPtr msg);
};

/**
 * @brief 云台扫描模式的行为树节点
 */
class GimbalScanMode : public BT::StatefulActionNode {
public:
    GimbalScanMode(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("scan_range", 1.57, "Scan range in radians (default: 90 degrees)"),
            BT::InputPort<double>("scan_speed", 0.5, "Scan speed in rad/s"),
            BT::InputPort<double>("scan_duration", 10.0, "Maximum scan duration in seconds")
        };
    }
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gimbal_cmd_pub_;
    rclcpp::Timer::SharedPtr scan_timer_;
    
    double scan_range_;
    double scan_speed_;
    double scan_duration_;
    double current_yaw_;
    bool scanning_right_;
    std::chrono::steady_clock::time_point start_time_;
    
    void setupROS();
    void scanTimerCallback();
};

} // namespace robot_behavior_tree

#endif // ROBOT_BEHAVIOR_TREE_GIMBAL_NODES_HPP
