cmake_minimum_required(VERSION 3.0.2)
project(esc_move_base_mapping)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  octomap_msgs
  octomap_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  laser_geometry
  pcl_ros
  pcl_conversions
  pedsim_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} 
  # pedsim_msgs
  DEPENDS octomap PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(esc_move_base_mapper src/octomap_ls_pc.cpp)
target_link_libraries(esc_move_base_mapper ${catkin_LIBRARIES} ${Boost_LIBRARIES})

