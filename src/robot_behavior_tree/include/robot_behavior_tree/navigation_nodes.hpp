#ifndef ROBOT_BEHAVIOR_TREE_NAVIGATION_NODES_HPP
#define ROBOT_BEHAVIOR_TREE_NAVIGATION_NODES_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rb21_navigation_controller_msgs/action/navigate_to_goal.hpp>

namespace robot_behavior_tree {

/**
 * @brief 导航到指定位置的行为树节点
 */
class NavigateToPosition : public BT::StatefulActionNode {
public:
    NavigateToPosition(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::Point>("target_position"),
            BT::InputPort<double>("tolerance", 0.1, "Position tolerance in meters"),
            BT::InputPort<double>("max_speed", 0.5, "Maximum linear speed"),
            BT::OutputPort<std::string>("result_message")
        };
    }
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    using NavigateToGoal = rb21_navigation_controller_msgs::action::NavigateToGoal;
    rclcpp_action::Client<NavigateToGoal>::SharedPtr nav_client_;
    rclcpp_action::ClientGoalHandle<NavigateToGoal>::SharedPtr goal_handle_;
    
    void setupROS();
    void goalResponseCallback(const rclcpp_action::ClientGoalHandle<NavigateToGoal>::SharedPtr& goal_handle);
    void feedbackCallback(rclcpp_action::ClientGoalHandle<NavigateToGoal>::SharedPtr,
                         const std::shared_ptr<const NavigateToGoal::Feedback> feedback);
    void resultCallback(const rclcpp_action::ClientGoalHandle<NavigateToGoal>::WrappedResult& result);
    
    bool goal_completed_ = false;
    bool goal_succeeded_ = false;
    std::string result_message_;
};

/**
 * @brief 获取当前位置的行为树节点
 */
class GetCurrentPosition : public BT::SyncActionNode {
public:
    GetCurrentPosition(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<geometry_msgs::msg::Point>("current_position")
        };
    }
    
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    geometry_msgs::msg::Point current_position_;
    bool position_received_ = false;
    
    void setupROS();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

/**
 * @brief 检查是否到达目标位置的条件节点
 */
class IsAtPosition : public BT::ConditionNode {
public:
    IsAtPosition(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::Point>("target_position"),
            BT::InputPort<geometry_msgs::msg::Point>("current_position"),
            BT::InputPort<double>("tolerance", 0.1, "Position tolerance in meters")
        };
    }
    
    BT::NodeStatus tick() override;

private:
    double calculateDistance(const geometry_msgs::msg::Point& p1, 
                           const geometry_msgs::msg::Point& p2);
};

/**
 * @brief 停止机器人运动的行为树节点
 */
class StopRobot : public BT::SyncActionNode {
public:
    StopRobot(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {};
    }
    
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    void setupROS();
};

/**
 * @brief 旋转到指定角度的行为树节点
 */
class RotateToAngle : public BT::StatefulActionNode {
public:
    RotateToAngle(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("target_angle", "Target angle in radians"),
            BT::InputPort<double>("angular_speed", 0.5, "Angular speed in rad/s"),
            BT::InputPort<double>("tolerance", 0.1, "Angle tolerance in radians")
        };
    }
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    double target_angle_;
    double angular_speed_;
    double tolerance_;
    double current_yaw_;
    bool rotation_started_ = false;
    
    void setupROS();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    double normalizeAngle(double angle);
    double getYawFromOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
};

} // namespace robot_behavior_tree

#endif // ROBOT_BEHAVIOR_TREE_NAVIGATION_NODES_HPP
