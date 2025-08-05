#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

// Include all behavior tree node headers
#include "robot_behavior_tree/navigation_nodes.hpp"
#include "robot_behavior_tree/gimbal_nodes.hpp"
#include "robot_behavior_tree/detection_nodes.hpp"

// ROS2 message types
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

namespace robot_behavior_tree {

class BehaviorTreeExecutor : public rclcpp::Node {
public:
    BehaviorTreeExecutor() : Node("behavior_tree_executor") {
        // Declare parameters
        this->declare_parameter("behavior_tree_file", "");
        this->declare_parameter("enable_logging", true);
        this->declare_parameter("log_file_path", "/tmp/bt_trace.fbl");
        this->declare_parameter("tick_frequency", 10.0);
        this->declare_parameter("auto_start", false);
        
        // Get parameters
        behavior_tree_file_ = this->get_parameter("behavior_tree_file").as_string();
        enable_logging_ = this->get_parameter("enable_logging").as_bool();
        log_file_path_ = this->get_parameter("log_file_path").as_string();
        tick_frequency_ = this->get_parameter("tick_frequency").as_double();
        auto_start_ = this->get_parameter("auto_start").as_bool();
        
        // Initialize BehaviorTree factory
        setupBehaviorTreeFactory();
        
        // Load behavior tree
        if (!behavior_tree_file_.empty()) {
            loadBehaviorTree(behavior_tree_file_);
        }
        
        // Setup ROS interfaces
        setupROSInterfaces();
        
        // Setup execution timer
        execution_timer_ = this->create_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / tick_frequency_)),
            std::bind(&BehaviorTreeExecutor::executionCallback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "BehaviorTree Executor initialized");
        
        if (auto_start_ && tree_) {
            startExecution();
        }
    }
    
    ~BehaviorTreeExecutor() {
        stopExecution();
    }

private:
    // Parameters
    std::string behavior_tree_file_;
    bool enable_logging_;
    std::string log_file_path_;
    double tick_frequency_;
    bool auto_start_;
    
    // BehaviorTree components
    BT::BehaviorTreeFactory factory_;
    std::unique_ptr<BT::Tree> tree_;
    std::unique_ptr<BT::StdCoutLogger> cout_logger_;
    std::unique_ptr<BT::MinitraceLogger> minitrace_logger_;
    std::unique_ptr<BT::FileLogger> file_logger_;
    
    // Execution control
    rclcpp::TimerBase::SharedPtr execution_timer_;
    bool is_running_ = false;
    
    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_service_;
    
    void setupBehaviorTreeFactory() {
        // Register navigation nodes
        factory_.registerNodeType<NavigateToPosition>("NavigateToPosition");
        factory_.registerNodeType<GetCurrentPosition>("GetCurrentPosition");
        factory_.registerNodeType<IsAtPosition>("IsAtPosition");
        factory_.registerNodeType<StopRobot>("StopRobot");
        factory_.registerNodeType<RotateToAngle>("RotateToAngle");
        
        // Register gimbal nodes
        factory_.registerNodeType<GimbalRotateTo>("GimbalRotateTo");
        factory_.registerNodeType<EnableGimbalTracking>("EnableGimbalTracking");
        factory_.registerNodeType<GimbalCenter>("GimbalCenter");
        factory_.registerNodeType<IsGimbalTracking>("IsGimbalTracking");
        factory_.registerNodeType<GimbalScanMode>("GimbalScanMode");
        
        // Register detection nodes
        factory_.registerNodeType<IsTargetDetected>("IsTargetDetected");
        factory_.registerNodeType<WaitForTarget>("WaitForTarget");
        factory_.registerNodeType<StartDetection>("StartDetection");
        factory_.registerNodeType<StopDetection>("StopDetection");
        factory_.registerNodeType<GetTargetImagePosition>("GetTargetImagePosition");
        factory_.registerNodeType<IsTargetCentered>("IsTargetCentered");
        
        // Register simple action for MoveForward (can be expanded)
        factory_.registerSimpleAction("MoveForward", 
            [this](BT::TreeNode& node) -> BT::NodeStatus {
                auto distance = node.getInput<double>("distance");
                auto max_speed = node.getInput<double>("max_speed");
                
                if (!distance || !max_speed) {
                    return BT::NodeStatus::FAILURE;
                }
                
                // This is a simplified implementation
                // In practice, you would use the chassis position controller
                RCLCPP_INFO(this->get_logger(), "Moving forward %.2f meters at %.2f m/s", 
                           distance.value(), max_speed.value());
                
                // Simulate movement time
                std::this_thread::sleep_for(std::chrono::milliseconds(
                    static_cast<int>(distance.value() / max_speed.value() * 1000)));
                
                return BT::NodeStatus::SUCCESS;
            });
        
        RCLCPP_INFO(this->get_logger(), "BehaviorTree factory setup completed");
    }
    
    void loadBehaviorTree(const std::string& file_path) {
        try {
            tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(file_path));
            
            // Setup loggers if enabled
            if (enable_logging_) {
                cout_logger_ = std::make_unique<BT::StdCoutLogger>(*tree_);
                minitrace_logger_ = std::make_unique<BT::MinitraceLogger>(*tree_, log_file_path_.c_str());
                file_logger_ = std::make_unique<BT::FileLogger>(*tree_, "bt_log.txt");
            }
            
            RCLCPP_INFO(this->get_logger(), "BehaviorTree loaded from: %s", file_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load BehaviorTree: %s", e.what());
        }
    }
    
    void setupROSInterfaces() {
        // Command subscriber
        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/behavior_tree/command", 10,
            std::bind(&BehaviorTreeExecutor::commandCallback, this, std::placeholders::_1));
        
        // Status publisher
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/behavior_tree/status", 10);
        
        // Control service
        control_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/behavior_tree/control",
            std::bind(&BehaviorTreeExecutor::controlServiceCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
    }
    
    void executionCallback() {
        if (!is_running_ || !tree_) {
            return;
        }
        
        try {
            BT::NodeStatus status = tree_->tickRoot();
            
            // Publish status
            auto status_msg = std_msgs::msg::String();
            switch (status) {
                case BT::NodeStatus::SUCCESS:
                    status_msg.data = "SUCCESS";
                    break;
                case BT::NodeStatus::FAILURE:
                    status_msg.data = "FAILURE";
                    break;
                case BT::NodeStatus::RUNNING:
                    status_msg.data = "RUNNING";
                    break;
                default:
                    status_msg.data = "UNKNOWN";
                    break;
            }
            status_pub_->publish(status_msg);
            
            // Handle completion
            if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
                RCLCPP_INFO(this->get_logger(), "BehaviorTree execution completed with status: %s", 
                           status_msg.data.c_str());
                // Could restart or stop based on configuration
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "BehaviorTree execution error: %s", e.what());
        }
    }
    
    void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string command = msg->data;
        
        if (command == "start") {
            startExecution();
        } else if (command == "stop") {
            stopExecution();
        } else if (command == "reset") {
            resetExecution();
        } else if (command.substr(0, 5) == "load:") {
            std::string file_path = command.substr(5);
            loadBehaviorTree(file_path);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
        }
    }
    
    void controlServiceCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (request->data) {
            startExecution();
            response->success = true;
            response->message = "BehaviorTree execution started";
        } else {
            stopExecution();
            response->success = true;
            response->message = "BehaviorTree execution stopped";
        }
    }
    
    void startExecution() {
        if (!tree_) {
            RCLCPP_ERROR(this->get_logger(), "No BehaviorTree loaded");
            return;
        }
        
        is_running_ = true;
        RCLCPP_INFO(this->get_logger(), "BehaviorTree execution started");
    }
    
    void stopExecution() {
        is_running_ = false;
        RCLCPP_INFO(this->get_logger(), "BehaviorTree execution stopped");
    }
    
    void resetExecution() {
        if (tree_) {
            tree_->haltTree();
            RCLCPP_INFO(this->get_logger(), "BehaviorTree reset");
        }
    }
};

} // namespace robot_behavior_tree

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<robot_behavior_tree::BehaviorTreeExecutor>();
    
    RCLCPP_INFO(node->get_logger(), "Starting BehaviorTree Executor...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
