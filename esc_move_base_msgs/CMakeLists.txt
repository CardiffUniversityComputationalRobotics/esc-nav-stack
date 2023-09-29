cmake_minimum_required(VERSION 2.8.3)
project(esc_move_base_msgs)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  roslib
  actionlib_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  Path2D.msg
)

add_action_files(
  FILES
  Goto2D.action
  GotoRegion2D.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs actionlib_msgs
  #DEPENDS Eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})
