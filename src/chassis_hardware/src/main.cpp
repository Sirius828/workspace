#include <rclcpp/rclcpp.hpp>
#include "chassis_hardware/chassis_hardware_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<chassis_hardware::ChassisHardwareNode>();
        
        RCLCPP_INFO(node->get_logger(), "Starting chassis hardware node...");
        
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("chassis_hardware_main"), 
                     "Exception in main: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
