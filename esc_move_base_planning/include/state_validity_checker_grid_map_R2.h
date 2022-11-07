/*! \file state_validity_checker_octomap_fcl_R2.hpp
 * \brief State validity checker.
 *
 * \date March 5, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
 *
 * \details Check is a given configuration R2 is collision-free.
 *  The workspace is represented by an Octomap and collision check is done with FCL.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#ifndef OMPL_CONTRIB_STATE_VALIDITY_CHECKER_GRID_MAP_R2_
#define OMPL_CONTRIB_STATE_VALIDITY_CHECKER_GRID_MAP_R2_

// ROS
#include <ros/ros.h>
// ROS markers rviz
#include <visualization_msgs/Marker.h>
// ROS messages
#include <nav_msgs/Odometry.h>
// ROS tf
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

// Standard libraries
#include <cstdlib>
#include <cmath>
#include <string>

// grid map library
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GridMap.h>

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
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// #include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <math.h>

// ROS-GridMap interface
using grid_map_msgs::GetGridMap;

// Standard namespace
using namespace std;

// OMPL namespaces
namespace ob = ompl::base;
namespace og = ompl::geometric;

//!  OmFclStateValidityCheckerR2 class.
/*!
  GridMap State Validity checker.
  Extension of an abstract class used to implement the state validity checker over a grid map.
*/
class OmFclStateValidityCheckerR2 : public ob::StateValidityChecker
{
public:
  //! OmFclStateValidityCheckerR2 constructor.
  /*!
   * Besides of initializing the private attributes, it loads the grid map.
   */
  OmFclStateValidityCheckerR2(const ob::SpaceInformationPtr &si, const bool opport_collision_check,
                              std::vector<double> planning_bounds_x, std::vector<double> planning_bounds_y);

  //! OmFclStateValidityCheckerR2 destructor.
  /*!
   * Destroy the grid map.
   */
  ~OmFclStateValidityCheckerR2();

  //! State validator.
  /*!
   * Function that verifies if the given state is valid (i.e. is free of collision) using FCL
   */
  virtual bool isValid(const ob::State *state) const;

  /*
   * Returns the cost value for the integration of the path defined on the equation that defines an extended
   * social comfort zone model.
   */
  virtual double checkExtendedSocialComfort(const ob::State *state,
                                            const ob::SpaceInformationPtr space) const;

  virtual bool isValidPoint(const ob::State *state) const;

  /*
   * Calculates the value of the interaction between robot agent and social agent
   */
  double basicPersonalSpaceFnc(const ob::State *state, const pedsim_msgs::AgentState agentState,
                               const ob::SpaceInformationPtr space) const;

  /*
   * Calculates the value of the interaction between robot agent and social agent as the extended social
   * model
   */
  double extendedPersonalSpaceFnc(const ob::State *state, const pedsim_msgs::AgentState agentState,
                                  const ob::SpaceInformationPtr space) const;

  /*
   * Calculates wether an agent is in the field of view of the robot.
   */
  bool isAgentInRFOV(const pedsim_msgs::AgentState agentState) const;

  bool isRobotInFront(const ob::State *state, const pedsim_msgs::AgentState agentState,
                      const ob::SpaceInformationPtr space) const;

private:
  // ROS
  ros::NodeHandle nh_, local_nh_;

  double grid_map_min_x_, grid_map_min_y_, grid_map_min_z_;
  double grid_map_max_x_, grid_map_max_y_, grid_map_max_z_;
  std::vector<double> planning_bounds_x_, planning_bounds_y_;
  double robot_base_radius_, robot_base_height_;
  std::string grid_map_service_;
  grid_map_msgs::GridMap grid_map_msgs_;
  grid_map::GridMap grid_map_;

  // cost objective type
  std::string optimization_objective;

  // topics
  std::string sim_agents_topic;
  std::string odometry_topic;

  // extra frames
  std::string main_frame;

  double octree_res_;

  bool opport_collision_check_;

  // relevant agents vector
  pedsim_msgs::AgentStatesConstPtr relevant_agent_states_;

  // odometry data
  nav_msgs::OdometryConstPtr odomData;

  //! basic social personal space parameters defined
  /*
   * amplitude of basic social personal space function
   */
  double ap = 100;

  /*
   * standard deviation in X of gaussian basic social personal space function
   */
  double sigma_x_ = 0.45;

  /*
   * standard deviation in X of gaussian basic social personal space function
   */
  double sigma_y_ = 0.45;

  //! extra parameters for social space model

  /*
   * normalization factor, multiplied by agent velocity
   */
  double fv = 0.8;

  /*
   * frontal area factor, sums with rest of factors
   */
  double frontal_factor_ = 0.2;

  /*
   * field of view factor, sums with rest of factors
   */
  double fov_factor_ = 0.0;

  //! agents parameters
  /*
   * agent fov angle
   */

  // /*
  //  * Angle defined when velocity is involved between robot and agent
  //  */
  // double angleMotionDir = 1;

  // /*
  //  * Gaze angle direction, specifically when agent is static
  //  */
  // double angleGazeDir = 1;

  //! parameters for robot field of view
  /*
   * This is the angle of field of view of the robot.
   */

  double angle_robot_view_ = 0.5 * M_PI;

  /*
   * This is the angle of field of view of the robot.
   */
  double robot_distance_view_ = 6;

  /*
   * This is the velocity that will create de maximum distance for agent evaluation
   */
  double robot_velocity_threshold_ = 0.38;
};

#endif
