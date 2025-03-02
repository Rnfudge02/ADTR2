cmake_minimum_required(VERSION 3.8)
project(adtr2_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#Core Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

#Standard communication interfaces
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

#ZED2 Drivers
find_package(zed_wrapper REQUIRED)
find_package(zed_ros2 REQUIRED)
find_package(zed_components REQUIRED)

#Navigation messages
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

#Add executables to build
add_library(auv-controller-component SHARED src/AUVController.cpp)
add_library(auv-monitor-component SHARED src/AUVMonitor.cpp)
add_library(auv-director-component SHARED src/AUVDirector.cpp)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${tf2_INCLUDE_DIRS}
)

#Set the compiling features for each target
target_compile_features(auv-controller-component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(auv-monitor-component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(auv-director-component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#Use ament to set target dependencies.
ament_target_dependencies(auv-controller-component rclcpp rclcpp_components std_srvs tf2 tf2_geometry_msgs mavros_msgs std_msgs)
ament_target_dependencies(auv-monitor-component rclcpp rclcpp_components std_msgs std_srvs std_msgs)
ament_target_dependencies(auv-director-component rclcpp rclcpp_components std_msgs std_srvs vision_msgs std_msgs)

#Register composable nodes
rclcpp_components_register_node(
  auv-controller-component
  PLUGIN "adtr2::controller::AUVController"
  EXECUTABLE auv-controller
)

rclcpp_components_register_node(
  auv-monitor-component
  PLUGIN "adtr2::monitor::AUVMonitor"
  EXECUTABLE auv-monitor
)

rclcpp_components_register_node(
  auv-director-component
  PLUGIN "adtr2::director::AUVDirector"
  EXECUTABLE auv-director
)

#Install targets to destination directory,
ament_export_targets(export-auv-controller-component)
install(TARGETS auv-controller-component
  EXPORT export-auv-controller-component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#Install targets to destination directory,
ament_export_targets(export-auv-monitor-component)
install(TARGETS auv-monitor-component
  EXPORT export-auv-monitor-component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#Install targets to destination directory,
ament_export_targets(export-auv-director-component)
install(TARGETS auv-director-component
  EXPORT export-auv-director-component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
