cmake_minimum_required(VERSION 3.8)
project(simple_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 包含头文件目录
include_directories(include)

# 创建简单导航器可执行文件
add_executable(simple_navigator src/simple_navigator.cpp)
ament_target_dependencies(simple_navigator 
  rclcpp 
  geometry_msgs 
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# 安装可执行文件
install(TARGETS
  simple_navigator
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Python脚本
install(
  PROGRAMS 
    scripts/send_goal.py
    scripts/send_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
