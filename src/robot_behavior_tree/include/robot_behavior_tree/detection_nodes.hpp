#ifndef ROBOT_BEHAVIOR_TREE_DETECTION_NODES_HPP
#define ROBOT_BEHAVIOR_TREE_DETECTION_NODES_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_behavior_tree {

/**
 * @brief 检查是否检测到目标的条件节点
 */
class IsTargetDetected : public BT::ConditionNode {
public:
    IsTargetDetected(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("target_class", "Target class to detect"),
            BT::InputPort<double>("min_confidence", 0.8, "Minimum detection confidence"),
            BT::InputPort<double>("timeout", 1.0, "Detection timeout in seconds"),
            BT::OutputPort<geometry_msgs::msg::Point>("target_position")
        };
    }
    
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    
    std::string target_class_;
    double min_confidence_;
    double timeout_;
    geometry_msgs::msg::Point last_target_position_;
    std::chrono::steady_clock::time_point last_detection_time_;
    bool target_detected_ = false;
    
    void setupROS();
    void targetCallback(const geometry_msgs::msg::Point::SharedPtr msg);
};

/**
 * @brief 等待目标检测的行为树节点
 */
class WaitForTarget : public BT::StatefulActionNode {
public:
    WaitForTarget(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("target_class", "Target class to wait for"),
            BT::InputPort<double>("min_confidence", 0.8, "Minimum detection confidence"),
            BT::InputPort<double>("max_wait_time", 30.0, "Maximum wait time in seconds"),
            BT::OutputPort<geometry_msgs::msg::Point>("target_position")
        };
    }
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    
    std::string target_class_;
    double min_confidence_;
    double max_wait_time_;
    geometry_msgs::msg::Point target_position_;
    std::chrono::steady_clock::time_point start_time_;
    bool target_found_ = false;
    
    void setupROS();
    void targetCallback(const geometry_msgs::msg::Point::SharedPtr msg);
};

/**
 * @brief 启动目标检测的行为树节点
 */
class StartDetection : public BT::SyncActionNode {
public:
    StartDetection(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("target_class", "ferrari", "Target class to detect"),
            BT::InputPort<double>("confidence_threshold", 0.8, "Detection confidence threshold")
        };
    }
    
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_cmd_pub_;
    
    void setupROS();
};

/**
 * @brief 停止目标检测的行为树节点
 */
class StopDetection : public BT::SyncActionNode {
public:
    StopDetection(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {};
    }
    
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_cmd_pub_;
    
    void setupROS();
};

/**
 * @brief 获取目标在图像中的位置信息
 */
class GetTargetImagePosition : public BT::SyncActionNode {
public:
    GetTargetImagePosition(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("target_class", "Target class"),
            BT::OutputPort<geometry_msgs::msg::Point>("pixel_position"),
            BT::OutputPort<double>("confidence")
        };
    }
    
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    
    geometry_msgs::msg::Point pixel_position_;
    double confidence_;
    bool position_received_ = false;
    std::chrono::steady_clock::time_point last_update_;
    
    void setupROS();
    void targetCallback(const geometry_msgs::msg::Point::SharedPtr msg);
};

/**
 * @brief 检查目标是否在图像中心的条件节点
 */
class IsTargetCentered : public BT::ConditionNode {
public:
    IsTargetCentered(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::Point>("target_pixel_position"),
            BT::InputPort<double>("center_tolerance", 50.0, "Center tolerance in pixels"),
            BT::InputPort<int>("image_width", 640, "Image width in pixels"),
            BT::InputPort<int>("image_height", 480, "Image height in pixels")
        };
    }
    
    BT::NodeStatus tick() override;

private:
    double calculatePixelDistance(const geometry_msgs::msg::Point& target_pos, 
                                 int image_width, int image_height);
};

} // namespace robot_behavior_tree

#endif // ROBOT_BEHAVIOR_TREE_DETECTION_NODES_HPP
