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

#ifndef OMPL_CONTRIB_STATE_VALIDITY_CHECKER_FCL_OCTOMAP_R2_
#define OMPL_CONTRIB_STATE_VALIDITY_CHECKER_FCL_OCTOMAP_R2_

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

// Octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

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

// FCL
#include <fcl/fcl.h>
#include <fcl/collision.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/broadphase/broadphase_spatialhash.h>
#include <fcl/common/types.h>
#include <fcl/config.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/math/geometry-inl.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

#include <iostream>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// #include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <math.h>

// ROS-Octomap interface
using octomap_msgs::GetOctomap;
// Standard namespace
using namespace std;
// Octomap namespace
using namespace octomap;
// OMPL namespaces
namespace ob = ompl::base;
namespace og = ompl::geometric;

//!  OmFclStateValidityCheckerR2 class.
/*!
  Octomap State Validity checker.
  Extension of an abstract class used to implement the state validity checker over an octomap using FCL.
*/
class OmFclStateValidityCheckerR2 : public ob::StateValidityChecker
{
public:
    //! OmFclStateValidityCheckerR2 constructor.
    /*!
     * Besides of initializing the private attributes, it loads the octomap.
     */
    OmFclStateValidityCheckerR2(const ob::SpaceInformationPtr &si, const bool opport_collision_check,
                                std::vector<double> planning_bounds_x, std::vector<double> planning_bounds_y);

    //! OmFclStateValidityCheckerR2 destructor.
    /*!
     * Destroy the octomap.
     */
    ~OmFclStateValidityCheckerR2();

    //! State validator.
    /*!
     * Function that verifies if the given state is valid (i.e. is free of collision) using FCL
     */
    virtual bool isValid(const ob::State *state) const;

    //! State clearance.
    /*!
     * Returns the minimum distance from the given robot state and the environment
     */
    virtual double clearance(const ob::State *state) const;

    virtual double checkRiskZones(const ob::State *state) const;

    /*
     * Returns the cost value for the integration of the path defined on the equation that defines social
     * comfort zone.
     */
    virtual double checkSocialComfort(const ob::State *state, const ob::SpaceInformationPtr space) const;

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
    bool isAgentInRFOV(const ob::State *state, const pedsim_msgs::AgentState agentState,
                       const ob::SpaceInformationPtr space) const;

    bool isRobotInFront(const ob::State *state, const pedsim_msgs::AgentState agentState,
                        const ob::SpaceInformationPtr space) const;

private:
    // ROS
    ros::NodeHandle nh_, local_nh_;

    // Octomap
    octomap::AbstractOcTree *abs_octree_;
    octomap::OcTree *octree_;
    double octree_min_x_, octree_min_y_, octree_min_z_;
    double octree_max_x_, octree_max_y_, octree_max_z_;
    std::vector<double> planning_bounds_x_, planning_bounds_y_;
    double pepper_base_radius_, pepper_base_height_;
    std::string octomap_service_;

    // cost objective type
    std::string optimization_objective;

    // topics
    std::string sim_agents_topic;
    std::string odometry_topic;

    // extra frames
    std::string main_frame;

    double octree_res_;

    // FCL
    fcl::OcTreef *tree_;
    fcl::CollisionObjectf *tree_obj_;
    std::shared_ptr<fcl::Cylinderf> pepper_collision_solid_;
    std::shared_ptr<fcl::Cylinderf> agent_collision_solid_;

    bool opport_collision_check_;

    // pedsim variables
    pedsim_msgs::AgentStatesConstPtr agentStates;

    // odometry data
    nav_msgs::OdometryConstPtr odomData;

    //! basic social personal space parameters defined
    /*
     * amplitude of basic social personal space function
     */
    double Ap = 100;

    /*
     * standard deviation in X of gaussian basic social personal space function
     */
    double sigmaX = 0.45;

    /*
     * standard deviation in X of gaussian basic social personal space function
     */
    double sigmaY = 0.45;

    // public:
    /*
     * distance between robot and agent
     */
    // double dRobotAgent = 1;
    // double tethaAgent;

    //! extra parameters for social space model

    /*
     * normalization factor, multiplied by agent velocity
     */
    double fv = 0.8;

    /*
     * frontal area factor, sums with rest of factors
     */
    double fFront = 0.2;

    /*
     * field of view factor, sums with rest of factors
     */
    double fFieldOfView = 0.0;

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

    // double fRobotView = (M_PI - ((M_PI - robotAngleView) * 2));
    double fRobotView = 0.5 * M_PI;

    /*
     * This is the angle of field of view of the robot.
     */
    double robotDistanceView = 6;

    /*
     * This is the velocity that will create de maximum distance for agent evaluation
     */
    double robotVelocityThreshold = 0.38;
};

#endif
