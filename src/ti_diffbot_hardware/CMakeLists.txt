cmake_minimum_required(VERSION 3.8)
project(ti_diffbot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 创建硬件接口库
add_library(${PROJECT_NAME} SHARED
  src/ti_diffbot_hardware.cpp
)

# 设置目标属性
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# 设置编译标志
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# 导出插件
pluginlib_export_plugin_description_file(hardware_interface ti_diffbot_hardware.xml)

# 安装库
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装插件描述文件
install(
  FILES ti_diffbot_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# 安装示例配置文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

# 安装launch文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# 安装URDF文件
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.xacro"
)

# 安装RViz配置文件
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.rviz"
)

# 安装脚本文件
install(
  PROGRAMS 
    scripts/test_diffbot.py
    scripts/test_chassis.py
    scripts/check_system.py
    scripts/diagnose_odom.py
    scripts/test_realtime.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 下面这行跳过版权检查，如果你想要检查版权可以注释掉
  set(ament_cmake_copyright_FOUND TRUE)
  # 下面这行跳过cpplint检查，如果你想要检查可以注释掉
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
