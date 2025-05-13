cmake_minimum_required(VERSION 3.5)
project(ins_processing)

# 查找必要的依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)  # 已经正确查找Eigen3
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED)

# 构建共享库，包含所有组件
add_library(gps_components SHARED
  src/gps_publisher.cpp
  src/gps_subscriber.cpp
  src/gps_converter.cpp
)

# 组件依赖
ament_target_dependencies(gps_components
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  Eigen3  # 确保Eigen3依赖已添加
  pluginlib
)

# 设置包含目录
target_include_directories(gps_components
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/ins_processing>
)

# 显式链接Eigen3库（加这个可能有助于解决问题）
target_link_libraries(gps_components Eigen3::Eigen)

# 注册组件
rclcpp_components_register_nodes(gps_components
  "ins_processing::GpsPublisher"
  "ins_processing::GpsSubscriber"
  "ins_processing::GpsConverter"
)

# 插件导出（pluginlib）
message(STATUS "Plugin description file: ${CMAKE_CURRENT_SOURCE_DIR}/plugin_description.xml")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

pluginlib_export_plugin_description_file(rclcpp_components plugin_description.xml)

# 安装共享库
install(TARGETS gps_components
  LIBRARY DESTINATION lib
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/ins_processing
)

# 安装插件描述文件
install(FILES plugin_description.xml
  DESTINATION share/ins_processing
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/ins_processing/launch
)

# 安装 package.xml
install(FILES package.xml
  DESTINATION share/ins_processing
)

ament_package()





























