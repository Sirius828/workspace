cmake_minimum_required(VERSION 3.8)
project(yolo_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectionBox.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 安装Python节点
install(PROGRAMS
  yolo_detector/detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_detector
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install modules
install(DIRECTORY modules/
  DESTINATION share/${PROJECT_NAME}/modules
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
