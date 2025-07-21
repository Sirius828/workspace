cmake_minimum_required(VERSION 3.8)
project(diffbot_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 添加可执行文件
add_executable(diffbot_service_node src/diffbot_service_node.cpp)

# 链接依赖
ament_target_dependencies(
  diffbot_service_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  controller_manager_msgs
)

# 安装可执行文件
install(TARGETS
  diffbot_service_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装启动文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# 安装脚本
install(
  PROGRAMS
    scripts/publish_number.py
    scripts/call_start_service.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
