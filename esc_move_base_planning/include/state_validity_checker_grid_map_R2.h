/*! \file state_validity_checker_grid_map_R2.hpp
 * \brief State validity checker.
 *
 * \date March 5, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
 *
 * \details Check is a given configuration R2 is collision-free.
 *  The workspace is represented by an GridMap and collision check is done iterating.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#ifndef OMPL_CONTRIB_STATE_VALIDITY_CHECKER_GRID_MAP_R2_
#define OMPL_CONTRIB_STATE_VALIDITY_CHECKER_GRID_MAP_R2_

// ROS2
#include <rclcpp/rclcpp.hpp>
// ROS2 messages
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Standard libraries
#include <cstdlib>
#include <cmath>
#include <string>

// grid map library
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/srv/get_grid_map.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

// OMPL
#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/debug/Profiler.h>

// Boost
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

// Eigen
#include <Eigen/Dense>

#include <iostream>
#include <pedsim_msgs/msg/agent_states.hpp>
#include <pedsim_msgs/msg/agent_state.hpp>

// #include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <math.h>

// ROS-GridMap interface
using grid_map_msgs::srv::GetGridMap;
// Standard namespace
using namespace std;

// OMPL namespaces
namespace ob = ompl::base;
namespace og = ompl::geometric;

//!  GridMapStateValidityCheckerR2 class.
/*!
  GridMap State Validity checker.
  Extension of an abstract class used to implement the state validity checker over a grid map.
*/
class GridMapStateValidityCheckerR2 : public ob::StateValidityChecker
{
public:
  //! GridMapStateValidityCheckerR2 constructor.
  /*!
   * Besides of initializing the private attributes, it loads the grid map.
   */
  GridMapStateValidityCheckerR2(const ob::SpaceInformationPtr &si, const bool opport_collision_check,
                                std::vector<double> planning_bounds_x, std::vector<double> planning_bounds_y, grid_map_msgs::msg::GridMap grid_map_msg, const double robot_radius);

  //! GridMapStateValidityCheckerR2 destructor.
  /*!
   * Destroy the grid map.
   */
  ~GridMapStateValidityCheckerR2();

  //! State validator.
  /*!
   * Function that verifies if the given state is valid (i.e. is free of collision) iterating the grid map
   */
  virtual bool isValid(const ob::State *state) const;

  /*
   * Returns the cost value for the integration of the path defined on the equation that defines an extended
   * social comfort zone model.
   */
  virtual double checkExtendedSocialComfort(const ob::State *state,
                                            const ob::SpaceInformationPtr space) const;

  virtual bool isValidPoint(const ob::State *state) const;

private:
  double grid_map_min_x_, grid_map_min_y_, grid_map_min_z_;
  double grid_map_max_x_, grid_map_max_y_, grid_map_max_z_;
  std::vector<double> planning_bounds_x_, planning_bounds_y_;
  double robot_base_radius_;
  std::string grid_map_service_;
  grid_map_msgs::msg::GridMap grid_map_msgs_;
  grid_map::GridMap grid_map_;

  // cost objective type
  std::string optimization_objective;

  bool opport_collision_check_;

  grid_map::Matrix obstacles_grid_map_;
  grid_map::Matrix comfort_grid_map_;
};

#endif
