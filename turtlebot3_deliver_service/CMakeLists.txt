################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_deliver_service)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  actionlib_msgs
  move_base_msgs
  tf2_msgs
  turtlebot3_msgs
  message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  PadOrder.msg
  AvailableItemList.msg
  ServiceStatus.msg
)

add_service_files(
  FILES
  InitTurtlebotPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    actionlib_msgs
    move_base_msgs
    tf2_msgs
    turtlebot3_msgs
    message_runtime
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(service_core src/service_core.cpp)
add_dependencies(service_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(service_core ${catkin_LIBRARIES})

add_executable(pad_virtual src/pad_virtual.cpp)
add_dependencies(pad_virtual ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pad_virtual ${catkin_LIBRARIES})

add_executable(pose_initialization src/pose_initialization.cpp)
add_dependencies(pose_initialization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_initialization ${catkin_LIBRARIES})

add_executable(call_initial_pose src/call_initial_pose.cpp)
add_dependencies(call_initial_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(call_initial_pose ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS service_core pad_virtual pose_initialization call_initial_pose
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch param 
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
