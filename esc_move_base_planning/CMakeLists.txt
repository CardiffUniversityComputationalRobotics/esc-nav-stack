cmake_minimum_required(VERSION 2.8.3)
project(esc_move_base_planning)

## Add support for C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
  tf2_eigen
  actionlib
  actionlib_msgs
  esc_move_base_msgs
  pedsim_msgs
  octomap_ros
)


find_package(Octomap REQUIRED)
find_package(octomap REQUIRED)
find_package(ompl REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)

find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp roslib rospy sensor_msgs std_msgs tf tf2 tf2_ros actionlib esc_move_base_msgs 
  # pedsim_msgs
  DEPENDS OMPL octomap
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)
link_directories(${OCTOMAP_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS} ${ompl_LIBRARY_DIRS} ${FCL_LIBRARY_DIRS} ${CCD_LIBRARY_DIRS})


add_executable(esc_move_base_planner
src/planner/RRTstarMod.cpp
src/planning_framework_main.cpp
src/state_validity_checker_octomap_fcl_R2.cpp
src/new_state_sampler.cpp
src/state_cost_objective.cpp
)
target_link_libraries(esc_move_base_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES} ccd ${OCTOMAP_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})
add_dependencies(esc_move_base_planner ${catkin_EXPORTED_TARGETS} esc_move_base_msgs_generate_messages_cpp ${esc_move_base_msgs_EXPORTED_TARGETS})

