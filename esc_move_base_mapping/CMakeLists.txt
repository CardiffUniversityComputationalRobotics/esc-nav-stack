cmake_minimum_required(VERSION 2.8.3)
project(esc_move_base_mapping)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-std=c++11)

set(octomap_DIR /home/sasm/octomap/lib/cmake/octomap)
set(OCTOMAP_LIBRARY_DIRS /home/sasm/octomap/lib)
set(octomap_INCLUDE_DIRS /home/sasm/octomap/octomap/include/octomap)


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


# find_package(octomap REQUIRED)
# include_directories(${OCTOMAP_INCLUDE_DIRS})
# link_libraries(${OCTOMAP_LIBRARIES})

find_package(octomap 1.9 REQUIRED)
include_directories(BEFORE ${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${PROJECT_NAME} /home/sasm/octomap/lib/liboctomap.so /home/sasm/octomap/lib/liboctomath.so)
MESSAGE(STATUS "OctoMap paths:" ${OCTOMAP_INCLUDE_DIRS} " " ${OCTOMAP_LIBRARY_DIRS})

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} 
  # pedsim_msgs
  DEPENDS octomap PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(esc_move_base_mapper src/octomap_ls_pc.cpp)
target_link_libraries(esc_move_base_mapper ${catkin_LIBRARIES} ${Boost_LIBRARIES})

