cmake_minimum_required(VERSION 2.8.3)
project(esc_move_base_mapping)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-std=c++11)

# set(social_octomap_DIR /home/sasm/social_octomap/lib/cmake/social_octomap)
# set(SOCIAL_OCTOMAP_LIBRARY_DIRS /home/sasm/social_octomap/lib)
# set(social_octomap_INCLUDE_DIRS /home/sasm/social_octomap/social_octomap/include)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  social_octomap_msgs
  social_octomap_ros
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

# find_package(social_octomap REQUIRED)
# include_directories(BEFORE ${SOCIAL_OCTOMAP_INCLUDE_DIRS})
# link_directories(${SOCIAL_OCTOMAP_LIBRARY_DIRS})
# link_libraries(${PROJECT_NAME} ${SOCIAL_OCTOMAP_LIBRARY_DIRS}/libsocial_octomap.so ${SOCIAL_OCTOMAP_LIBRARY_DIRS}/liboctomath.so)
# MESSAGE(STATUS "OctoMap paths:" ${SOCIAL_OCTOMAP_INCLUDE_DIRS} " " ${SOCIAL_OCTOMAP_LIBRARY_DIRS})

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} 
  # pedsim_msgs
  DEPENDS social_octomap PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(esc_move_base_mapper src/octomap_ls_pc.cpp)
target_link_libraries(esc_move_base_mapper ${catkin_LIBRARIES} ${Boost_LIBRARIES})

