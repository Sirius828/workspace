cmake_minimum_required(VERSION 3.8)
project(robot_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(behaviortree_cpp_v3 REQUIRED)  # Comment out for now
# find_package(vision_services_msgs REQUIRED)  # Comment out for now
# find_package(rb21_navigation_controller_msgs REQUIRED)  # Comment out for now

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RobotMission.action"
  "action/PatrolMission.action"
  "action/SearchAndTrack.action"
  "msg/BehaviorTreeStatus.msg"
  "msg/MissionCommand.msg"
  "srv/StartMission.srv"
  "srv/StopMission.srv"
  DEPENDENCIES geometry_msgs std_msgs action_msgs builtin_interfaces
)

# Include directories
include_directories(include)

# Comment out the complex nodes library for now - build interfaces first
# BehaviorTree nodes library
# add_library(${PROJECT_NAME}_nodes SHARED
#   src/behavior_tree_nodes/navigation_nodes.cpp
#   src/behavior_tree_nodes/gimbal_nodes.cpp
#   src/behavior_tree_nodes/detection_nodes.cpp
#   src/behavior_tree_nodes/composite_nodes.cpp
# )

# ament_target_dependencies(${PROJECT_NAME}_nodes
#   rclcpp
#   rclcpp_action
#   std_msgs
#   geometry_msgs
#   nav_msgs
#   sensor_msgs
#   tf2
#   tf2_ros
#   behaviortree_cpp_v3
#   vision_services_msgs
#   rb21_navigation_controller_msgs
# )

# Comment out executables for now
# Main behavior tree executor
# add_executable(behavior_tree_executor
#   src/behavior_tree_executor.cpp
# )

# ament_target_dependencies(behavior_tree_executor
#   rclcpp
#   rclcpp_action
#   std_msgs
#   geometry_msgs
#   nav_msgs
#   behaviortree_cpp_v3
# )

# target_link_libraries(behavior_tree_executor ${PROJECT_NAME}_nodes)

# Mission planner
# add_executable(mission_planner
#   src/mission_planner.cpp
# )

# ament_target_dependencies(mission_planner
#   rclcpp
#   rclcpp_action
#   std_msgs
#   geometry_msgs
# )

# Comment out install targets for now
# Install targets
# install(TARGETS
#   ${PROJECT_NAME}_nodes
#   behavior_tree_executor
#   mission_planner
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install behavior tree XML files
install(DIRECTORY
  behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate interface dependencies
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(${PROJECT_NAME}_nodes "${cpp_typesupport_target}")  # Comment out for now

ament_package()
