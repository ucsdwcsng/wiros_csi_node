cmake_minimum_required(VERSION 3.3)
project(wiros_csi_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -g)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

## optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
    message("Building in DEBUG mode.")
    add_compile_definitions(DEBUG_CSI)
ENDIF(CMAKE_BUILD_TYPE MATCHES DEBUG)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  rf_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ConfigureCSI.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
   # INCLUDE_DIRS utils
   # LIBRARIES utils
   CATKIN_DEPENDS std_msgs sensor_msgs roscpp rospy message_runtime rf_msgs
#  DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(
# include
        ${catkin_INCLUDE_DIRS}
	src
        rf_msgs

)

add_executable(csi_node src/shell_utils.cpp src/ros_interface.cpp src/wiros_csi_main.cpp)
#add_executable(ap_scanner src/apscanner.cpp)

add_dependencies(csi_node ${catkin_EXPORTED_TARGETS} wiros_csi_node_generate_messages_cpp)
#add_dependencies(ap_scanner ${catkin_EXPORTED_TARGETS} wiros_csi_node_generate_messages_cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(csi_node
   ${catkin_LIBRARIES}
 )
#target_link_libraries(ap_scanner
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/csi_rosbag_info.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS csi_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

#install(TARGETS ap_scanner
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

