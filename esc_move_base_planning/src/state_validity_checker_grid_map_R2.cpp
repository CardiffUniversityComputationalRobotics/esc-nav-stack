/*! \file state_validity_checker_grid_map_R2.cpp
 * \brief State validity checker.
 *
 * \date November 07, 2022
 * \author Juan David Hernandez Vega, HernandezVegaJ@cardiff.ac.uk
 * \author Steven Alexander Silva Mendoza, silvas1@cardiff.ac.uk
 *
 * \details Check is a given configuration R2 is collision-free.
 *  The workspace is represented by an GridMap and collision check is done by iterating.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#include <state_validity_checker_grid_map_R2.h>

GridMapStateValidityCheckerR2::GridMapStateValidityCheckerR2(const ob::SpaceInformationPtr &si,
                                                             const bool opport_collision_check,
                                                             std::vector<double> planning_bounds_x,
                                                             std::vector<double> planning_bounds_y)
    : ob::StateValidityChecker(si), local_nh_("~"), robot_base_radius_(0.4)
{
    GetGridMap::Request req;
    GetGridMap::Response resp;

    opport_collision_check_ = opport_collision_check;
    planning_bounds_x_ = planning_bounds_x;
    planning_bounds_y_ = planning_bounds_y;

    local_nh_.param("robot_base_radius", robot_base_radius_, robot_base_radius_);
    local_nh_.param("grid_map_service", grid_map_service_, grid_map_service_);
    local_nh_.param("social_agents_topic", social_agents_topic, social_agents_topic);
    local_nh_.param("odometry_topic", odometry_topic, odometry_topic);

    // ! GRID MAP REQUEST

    ROS_DEBUG("%s: requesting the map to %s...", ros::this_node::getName().c_str(),
              nh_.resolveName(grid_map_service_).c_str());

    ros::service::call(grid_map_service_, req, resp);

    if (grid_map::GridMapRosConverter::fromMessage(resp.map, grid_map_))
    {
        ROS_DEBUG("Obtained gridmap successfully");
        grid_map_msgs_ = resp.map;

        grid_map_max_x_ = grid_map_msgs_.info.pose.position.x + (grid_map_msgs_.info.length_x / 2);
        grid_map_min_x_ = grid_map_msgs_.info.pose.position.x - (grid_map_msgs_.info.length_x / 2);

        grid_map_max_y_ = grid_map_msgs_.info.pose.position.y + (grid_map_msgs_.info.length_y / 2);
        grid_map_min_y_ = grid_map_msgs_.info.pose.position.y - (grid_map_msgs_.info.length_y / 2);
    }
    else
    {
        ROS_ERROR("Error reading GridMap");
    }

    // ! SOCIAL AGENTS DATA RETRIEVE

    ROS_INFO_STREAM("Retrieving data from social agents.");
    relevant_agent_states_ = ros::topic::waitForMessage<pedsim_msgs::AgentStates>(social_agents_topic);

    // ! ODOMETRY DATA RETRIEVE

    ROS_INFO_STREAM("Retrieving robot odometry.");
    odomData = ros::topic::waitForMessage<nav_msgs::Odometry>(odometry_topic);
}

bool GridMapStateValidityCheckerR2::isValid(const ob::State *state) const
{
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    // ompl::tools::Profiler::Begin("collision");

    // extract the component of the state and cast it to what we expect

    if (opport_collision_check_ &&
        (state_r2->values[0] < grid_map_min_x_ || state_r2->values[1] < grid_map_min_y_ ||
         state_r2->values[0] > grid_map_max_x_ || state_r2->values[1] > grid_map_max_y_))
    {
        // ompl::tools::Profiler::End("collision");
        return true;
    }

    if (state_r2->values[0] < planning_bounds_x_[0] || state_r2->values[1] < planning_bounds_y_[0] ||
        state_r2->values[0] > planning_bounds_x_[1] || state_r2->values[1] > planning_bounds_y_[1])
    {
        // ompl::tools::Profiler::End("collision");
        return false;
    }

    grid_map::Position query(state_r2->values[0], state_r2->values[1]);

    for (grid_map::CircleIterator iterator(grid_map_, query, robot_base_radius_);
         !iterator.isPastEnd(); ++iterator)
    {

        if (grid_map_.at("full", *iterator) > 50)
        {
            return false;
        }
    }

    return true;
}

double GridMapStateValidityCheckerR2::checkExtendedSocialComfort(const ob::State *state,
                                                                 const ob::SpaceInformationPtr space) const
{
    double state_risk = 0.0;
    // double current_state_risk = 0.0;

    // for (int i = 0; i < relevant_agent_states_->agent_states.size(); i++)
    // {
    //     current_state_risk = this->extendedPersonalSpaceFnc(state, relevant_agent_states_->agent_states[i], space);

    //     if (current_state_risk > state_risk)
    //         state_risk = current_state_risk;
    // }

    // if (state_risk <= 1)
    //     state_risk = 1;

    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    grid_map::Position query(state_r2->values[0], state_r2->values[1]);

    state_risk = grid_map_.atPosition("comfort", query);

    if (state_risk < 1)
    {
        state_risk = 1;
    }

    return state_risk;
}

double GridMapStateValidityCheckerR2::extendedPersonalSpaceFnc(const ob::State *state,
                                                               const pedsim_msgs::AgentState agentState,
                                                               const ob::SpaceInformationPtr space) const
{
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    double dRobotAgent = std::sqrt(std::pow(agentState.pose.position.x - state_r2->values[0], 2) +
                                   std::pow(agentState.pose.position.y - state_r2->values[1], 2));

    double tethaRobotAgent = atan2((state_r2->values[1] - agentState.pose.position.y),
                                   (state_r2->values[0] - agentState.pose.position.x));

    if (tethaRobotAgent < 0)
    {
        tethaRobotAgent = 2 * M_PI + tethaRobotAgent;
    }

    double tethaOrientation;
    if (abs(agentState.twist.linear.x) > 0 || abs(agentState.twist.linear.y) > 0)
    {
        tethaOrientation = atan2(agentState.twist.linear.y, agentState.twist.linear.x);

        // tethaOrientation = angleMotionDir;
    }
    else
    {
        tf::Quaternion q(agentState.pose.orientation.x, agentState.pose.orientation.y,
                         agentState.pose.orientation.z, agentState.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        tethaOrientation = yaw;
    }

    if (tethaOrientation < 0)
    {
        tethaOrientation = 2 * M_PI + tethaOrientation;
    }

    bool robotInFront = false;
    bool robotInFOV = false;
    double modSigmaY;
    double agentVelocity;

    agentVelocity =
        std::sqrt(std::pow(agentState.twist.linear.x, 2) + std::pow(agentState.twist.linear.y, 2));

    robotInFront = this->isRobotInFront(state, agentState, space);

    if (robotInFront)
    {
        if (robotInFOV)
            modSigmaY = (1 + agentVelocity * fv + frontal_factor_ + fov_factor_) * sigma_y_;
        else
            modSigmaY = (1 + agentVelocity * fv + frontal_factor_) * sigma_y_;
    }
    else
    {
        modSigmaY = sigma_y_;
    }

    double basicPersonalSpaceVal =
        ap *
        std::exp(-(
            std::pow(dRobotAgent * std::cos(tethaRobotAgent - tethaOrientation) / (std::sqrt(2) * sigma_x_),
                     2) +
            std::pow(dRobotAgent * std::sin(tethaRobotAgent - tethaOrientation) / (std::sqrt(2) * modSigmaY),
                     2)));

    return basicPersonalSpaceVal;
}

bool GridMapStateValidityCheckerR2::isRobotInFront(const ob::State *state,
                                                   const pedsim_msgs::AgentState agentState,
                                                   const ob::SpaceInformationPtr space) const

{
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    double tethaAgentRobot = atan2((state_r2->values[1] - agentState.pose.position.y),
                                   (state_r2->values[0] - agentState.pose.position.x));

    if (tethaAgentRobot < 0)
    {
        tethaAgentRobot = 2 * M_PI + tethaAgentRobot;
    }

    tf::Quaternion q(agentState.pose.orientation.x, agentState.pose.orientation.y,
                     agentState.pose.orientation.z, agentState.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double agentAngle = yaw;

    if (agentAngle < 0)
    {
        agentAngle = 2 * M_PI + agentAngle;
    }

    if (tethaAgentRobot > (agentAngle + M_PI))
        tethaAgentRobot = abs(agentAngle + 2 * M_PI - tethaAgentRobot);
    else if (agentAngle > (tethaAgentRobot + M_PI))
        tethaAgentRobot = abs(tethaAgentRobot + 2 * M_PI - agentAngle);
    else
        tethaAgentRobot = abs(tethaAgentRobot - agentAngle);

    if (abs(tethaAgentRobot) < 0.5 * M_PI)
        return true;

    return false;
}

bool GridMapStateValidityCheckerR2::isValidPoint(const ob::State *state) const
{

    // extract the component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    grid_map::Position query(state_r2->values[0], state_r2->values[1]);

    if (grid_map_.atPosition("full", query) > 50)
    {
        return false;
    }

    return true;
}

GridMapStateValidityCheckerR2::~GridMapStateValidityCheckerR2()
{
}
