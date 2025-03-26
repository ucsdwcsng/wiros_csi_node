cmake_minimum_required(VERSION 3.8)
project(wiros_csi)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++20 -g -DWIROS_ROS2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

## optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
    message("Building in DEBUG mode.")
    add_compile_definitions(DEBUG_CSI)
ENDIF(CMAKE_BUILD_TYPE MATCHES DEBUG)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rf_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#add_service_files(
#  FILES
#  ConfigureCSI.srv
#)
include_directories(
        ${catkin_INCLUDE_DIRS}
        src
        rf_msgs
)

add_executable(csi_node src/shell_utils.cpp src/ros2_interface.cpp src/wiros_csi_main.cpp src/nex_conf.cpp)
ament_target_dependencies(csi_node rclcpp std_msgs rf_msgs)
install(TARGETS
  csi_node
  DESTINATION lib/${PROJECT_NAME})
#install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
