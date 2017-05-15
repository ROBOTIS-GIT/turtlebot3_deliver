cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_carrier_icra2017)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  turtlebot3_msgs
  genmsg
)

add_message_files(DIRECTORY msg
  FILES
  PadOrder.msg
  AvailableItemList.msg
  ServiceStatus.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES key_to_vel_pkg
# CATKIN_DEPENDS roscpp rospy
# DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(service_core src/service_core.cpp)
target_link_libraries(service_core ${catkin_LIBRARIES})
add_dependencies(service_core turtlebot3_carrier_icra2017_generate_messages_cpp)

add_executable(pad_virtual src/pad_virtual.cpp)
target_link_libraries(pad_virtual ${catkin_LIBRARIES})
add_dependencies(pad_virtual turtlebot3_carrier_icra2017_generate_messages_cpp)
