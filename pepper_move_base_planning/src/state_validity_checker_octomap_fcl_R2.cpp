/*! \file state_validity_checker_octomap_fcl_R2.cpp
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

#include <state_validity_checker_octomap_fcl_R2.h>

OmFclStateValidityCheckerR2::OmFclStateValidityCheckerR2(const ob::SpaceInformationPtr &si,
                                                         const bool opport_collision_check,
                                                         std::vector<double> planning_bounds_x,
                                                         std::vector<double> planning_bounds_y)
  : ob::StateValidityChecker(si), local_nh_("~"), pepper_base_radius_(0.4), pepper_base_height_(2.0)
{
    GetOctomap::Request req;
    GetOctomap::Response resp;

    opport_collision_check_ = opport_collision_check;
    planning_bounds_x_ = planning_bounds_x;
    planning_bounds_y_ = planning_bounds_y;

    local_nh_.param("pepper_base_radius", pepper_base_radius_, pepper_base_radius_);
    local_nh_.param("pepper_base_height", pepper_base_height_, pepper_base_height_);
    local_nh_.param("octomap_service", octomap_service_, octomap_service_);
    local_nh_.param("sim_agents_topic", sim_agents_topic, sim_agents_topic);
    local_nh_.param("odometry_topic", odometry_topic, odometry_topic);
    local_nh_.param("main_frame", main_frame, main_frame);
    local_nh_.param("optimization_objective", optimization_objective, optimization_objective);

    octree_ = NULL;

    ROS_DEBUG("%s: requesting the map to %s...", ros::this_node::getName().c_str(),
              nh_.resolveName(octomap_service_).c_str());

    while ((nh_.ok() && !ros::service::call(octomap_service_, req, resp)) || resp.map.data.size() == 0)
    {
        ROS_WARN("Request to %s failed; trying again...", nh_.resolveName(octomap_service_).c_str());
        usleep(1000000);
    }
    if (nh_.ok())
    {  // skip when CTRL-C
        abs_octree_ = octomap_msgs::msgToMap(resp.map);
        std::cout << std::endl;
        if (abs_octree_)
        {
            octree_ = dynamic_cast<octomap::OcTree *>(abs_octree_);
            tree_ = new fcl::OcTreef(std::shared_ptr<const octomap::OcTree>(octree_));
            tree_obj_ = new fcl::CollisionObjectf((std::shared_ptr<fcl::CollisionGeometryf>(tree_)));
        }

        pepper_collision_solid_.reset(new fcl::Cylinderf(pepper_base_radius_, pepper_base_height_));

        agent_collision_solid_.reset(new fcl::Cylinderf(0.35, pepper_base_height_));

        octree_res_ = octree_->getResolution();
        octree_->getMetricMin(octree_min_x_, octree_min_y_, octree_min_z_);
        octree_->getMetricMax(octree_max_x_, octree_max_y_, octree_max_z_);

        if (octree_)
            ROS_DEBUG("%s: Octomap received (%zu nodes, %f m res)", ros::this_node::getName().c_str(),
                      octree_->size(), octree_->getResolution());
        else
            ROS_ERROR("Error reading OcTree from stream");
    }

    ROS_INFO_STREAM("Retrieving data from social agents.");
    agentStates = ros::topic::waitForMessage<pedsim_msgs::AgentStates>(sim_agents_topic);
    // ROS_INFO_STREAM("Data from social agents: " << agentStates->agent_states[0].pose.position.x);

    ROS_INFO_STREAM("Retrieving robot odometry.");
    odomData = ros::topic::waitForMessage<nav_msgs::Odometry>(odometry_topic);

    // ROS_INFO_STREAM("Retrieving robot odometry.");
    // odomData = ros::topic::waitForMessage<nav_msgs::Odometry>(odometry_topic);

    // tl.transformPose(main_frame, msg_hand, rel_hand_pos);
}

bool OmFclStateValidityCheckerR2::isValid(const ob::State *state) const
{
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    // ompl::tools::Profiler::Begin("collision");

    // extract the component of the state and cast it to what we expect

    if (opport_collision_check_ &&
        (state_r2->values[0] < octree_min_x_ || state_r2->values[1] < octree_min_y_ ||
         state_r2->values[0] > octree_max_x_ || state_r2->values[1] > octree_max_y_))
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

    // FCL
    fcl::Transform3f pepper_tf;
    pepper_tf.setIdentity();
    pepper_tf.translate(fcl::Vector3f(state_r2->values[0], state_r2->values[1], pepper_base_height_ / 2.0));
    // fcl::Quaternion3f qt0;
    // qt0.fromEuler(0.0, 0.0, 0.0);
    // pepper_tf.setQuatRotation(qt0);

    fcl::CollisionObjectf vehicle_co(pepper_collision_solid_, pepper_tf);

    fcl::CollisionRequestf collision_request;
    fcl::CollisionResultf collision_result;

    fcl::collide(tree_obj_, &vehicle_co, collision_request, collision_result);

    // std::cout << "Collision (FCL): " << collision_result.isCollision() << std::endl;

    if (collision_result.isCollision())
    {
        // ompl::tools::Profiler::End("collision");
        return false;
    }

    //  agents collision checking

    double robotVelocity =
        std::sqrt(std::pow(odomData->twist.twist.linear.x, 2) + std::pow(odomData->twist.twist.linear.y, 2));

    double actualFOVDistance = robotDistanceView / robotVelocityThreshold * robotVelocity;

    if (actualFOVDistance < 1.5)
    {
        actualFOVDistance = 1.5;
    }

    // ROS_INFO_STREAM("distance of robot view " << actualFOVDistance);

    for (int i = 0; i < agentStates->agent_states.size(); i++)
    {
        pedsim_msgs::AgentState agentState = agentStates->agent_states[i];

        if (optimization_objective == "SocialComfort")
        {
            double dRobotAgent =
                std::sqrt(std::pow(agentState.pose.position.x - odomData->pose.pose.position.x, 2) +
                          std::pow(agentState.pose.position.y - odomData->pose.pose.position.y, 2));
          
          
            if (dRobotAgent < actualFOVDistance)
            {
                // FCL
                fcl::Transform3f agent_tf;
                agent_tf.setIdentity();
                agent_tf.translate(fcl::Vector3f(agentState.pose.position.x, agentState.pose.position.y,
                                                   pepper_base_height_ / 2.0));
                // fcl::Quaternion3f qt0;
                // qt0.fromEuler(0.0, 0.0, 0.0);
                // agent_tf.setQuatRotation(qt0);

                fcl::CollisionObjectf agent_co(agent_collision_solid_, agent_tf);
                fcl::collide(&agent_co, &vehicle_co, collision_request, collision_result);

                if (collision_result.isCollision())
                {
                    return false;
                }
            }
        }
        else
        {
            // FCL
            fcl::Transform3f agent_tf;
            agent_tf.setIdentity();
            agent_tf.translate(
                fcl::Vector3f(agentState.pose.position.x, agentState.pose.position.y, pepper_base_height_ / 2.0));
            // fcl::Quaternion3f qt0;
            // qt0.fromEuler(0.0, 0.0, 0.0);
            // agent_tf.setQuatRotation(qt0);

            fcl::CollisionObjectf agent_co(agent_collision_solid_, agent_tf);
            fcl::collide(&agent_co, &vehicle_co, collision_request, collision_result);

            if (collision_result.isCollision())
            {
                return false;
            }
        }
    }

    return true;
}

double OmFclStateValidityCheckerR2::clearance(const ob::State *state) const
{
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();
    double minDist = std::numeric_limits<double>::infinity();

    // ompl::tools::Profiler::Begin("clearance");

    // extract the component of the state and cast it to what we expect

    if (opport_collision_check_ &&
        (state_r2->values[0] < octree_min_x_ || state_r2->values[1] < octree_min_y_ ||
         state_r2->values[0] > octree_max_x_ || state_r2->values[1] > octree_max_y_))
    {
        // ompl::tools::Profiler::End("clearance");
        return minDist;
    }

    // FCL
    fcl::Transform3f vehicle_tf;
    vehicle_tf.setIdentity();
    vehicle_tf.translate(fcl::Vector3f(state_r2->values[0] + 3.5, state_r2->values[1], 0.0));
    // fcl::Quaternion3f qt0;
    // qt0.fromEuler(0.0, 0.0, 0.0);
    // vehicle_tf.setQuatRotation(qt0);

    fcl::CollisionObjectf vehicle_co(pepper_collision_solid_, vehicle_tf);
    fcl::DistanceRequestf distanceRequest;
    fcl::DistanceResultf distanceResult;

    fcl::distance(tree_obj_, &vehicle_co, distanceRequest, distanceResult);

    // std::cout << "Distance (FCL): " << distanceResult.min_distance << std::endl;

    if (distanceResult.min_distance < minDist)
        minDist = distanceResult.min_distance;

    // ompl::tools::Profiler::End("clearance");

    return minDist;
}

double OmFclStateValidityCheckerR2::checkRiskZones(const ob::State *state) const
{
    ROS_INFO_STREAM("running risk zone function");
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();
    double state_risk = 1.0;

    // ompl::tools::Profiler::Begin("RiskZones");

    // extract the component of the state and cast it to what we expect

    if (opport_collision_check_ &&
        (state_r2->values[0] < octree_min_x_ || state_r2->values[1] < octree_min_y_ ||
         state_r2->values[0] > octree_max_x_ || state_r2->values[1] > octree_max_y_))
    {
        // ompl::tools::Profiler::End("RiskZones");
        return state_risk;
    }

    // FCL
    fcl::Transform3f pepper_tf;
    pepper_tf.setIdentity();
    pepper_tf.translate(fcl::Vector3f(state_r2->values[0], state_r2->values[1], 0.0));
    // fcl::Quaternion3f qt0;
    // qt0.fromEuler(0.0, 0.0, 0.0);
    // pepper_tf.setQuatRotation(qt0);

    std::shared_ptr<fcl::Cylinderf> cyl0(new fcl::Cylinderf(pepper_base_radius_ + 0.2, pepper_base_height_));

    fcl::CollisionObjectf cyl0_co(cyl0, pepper_tf);
    fcl::CollisionRequestf collision_request;
    fcl::CollisionResultf collision_result;

    fcl::collide(tree_obj_, &cyl0_co, collision_request, collision_result);

    if (collision_result.isCollision())
        state_risk = 10.0;  // 15, 30
    else
    {
        std::shared_ptr<fcl::Cylinderf> cyl1(new fcl::Cylinderf(pepper_base_radius_ + 0.4, pepper_base_height_));
        fcl::CollisionObjectf cyl1_co(cyl1, pepper_tf);
        collision_result.clear();

        fcl::collide(tree_obj_, &cyl1_co, collision_request, collision_result);
        if (collision_result.isCollision())
            state_risk = 5.0;  // 10, 20
    }

    // ompl::tools::Profiler::End("RiskZones");

    return state_risk;
}

/*
 * Checks and returns the cost value of the robot according to the equation of social comfort.
 */
double OmFclStateValidityCheckerR2::checkSocialComfort(const ob::State *state,
                                                       const ob::SpaceInformationPtr space) const
{
    // ROS_INFO_STREAM("Running social comfort model");

    // const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();
    double state_risk = 0.0;

    for (int i = 0; i < agentStates->agent_states.size(); i++)
    {
        state_risk += this->basicPersonalSpaceFnc(state, agentStates->agent_states[i], space);
    }

    if (state_risk <= 1)
        state_risk = 1;

    // ROS_INFO_STREAM("The current state risk: " << state_risk);

    return state_risk;
}

double OmFclStateValidityCheckerR2::checkExtendedSocialComfort(const ob::State *state,
                                                               const ob::SpaceInformationPtr space) const
{
    // const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();
    double state_risk = 0.0;
    double current_state_risk = 0.0;

    // ROS_INFO_STREAM("Running extended social comfort model function");

    for (int i = 0; i < agentStates->agent_states.size(); i++)
    {
        if (this->isAgentInRFOV(state, agentStates->agent_states[i], space))
        {
            // ROS_INFO_STREAM("Agent in fov: " << agentStates->agent_states[i].id);
            current_state_risk = this->extendedPersonalSpaceFnc(state, agentStates->agent_states[i], space);
            // ROS_INFO_STREAM("agent risk: " << current_state_risk);
        }
        if (current_state_risk > state_risk)
            state_risk = current_state_risk;
    }

    if (state_risk <= 1)
        state_risk = 1;

    // ROS_INFO_STREAM("The current state risk: " << state_risk);

    // ROS_INFO_STREAM("agent risk: " << state_risk);

    return state_risk;
}

double OmFclStateValidityCheckerR2::basicPersonalSpaceFnc(const ob::State *state,
                                                          const pedsim_msgs::AgentState agentState,
                                                          const ob::SpaceInformationPtr space) const
{
    // const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    ob::ScopedState<> agentTf(space);
    agentTf[0] = double(agentState.pose.position.x);  // x
    agentTf[1] = double(agentState.pose.position.y);  // y

    double dRobotAgent = space->distance(state, agentTf->as<ob::State>());

    // double tethaRobotAgent = atan2((state_r2->values[1] - agentState.pose.position.y),
    //                                (state_r2->values[0] - agentState.pose.position.x));

    // double tethaOrientation;
    // if (abs(agentState.twist.linear.x) > 0 || abs(agentState.twist.linear.y) > 0)
    //     tethaOrientation = angleMotionDir;
    // else
    //     tethaOrientation = angleGazeDir;

    double tethaRobotAgent = atan2((odomData->pose.pose.position.y - agentState.pose.position.y),
                                   (odomData->pose.pose.position.x - agentState.pose.position.x));

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

    double basicPersonalSpaceVal =
        Ap *
        std::exp(
            -(std::pow(dRobotAgent * std::cos(tethaRobotAgent - tethaOrientation) / (std::sqrt(2) * sigmaX),
                       2) +
              std::pow(dRobotAgent * std::cos(tethaRobotAgent - tethaOrientation) / (std::sqrt(2) * sigmaY),
                       2)));

    return basicPersonalSpaceVal;
}

double OmFclStateValidityCheckerR2::extendedPersonalSpaceFnc(const ob::State *state,
                                                             const pedsim_msgs::AgentState agentState,
                                                             const ob::SpaceInformationPtr space) const
{
    // const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    // ob::ScopedState<> agentTf(space);
    // agentTf[0] = double(agentState.pose.position.x);  // x
    // agentTf[1] = double(agentState.pose.position.y);  // y

    // double dRobotAgent = space->distance(state, agentTf->as<ob::State>());

    double dRobotAgent = std::sqrt(std::pow(agentState.pose.position.x - odomData->pose.pose.position.x, 2) +
                                   std::pow(agentState.pose.position.y - odomData->pose.pose.position.y, 2));

    // double tethaRobotAgent = atan2((state_r2->values[1] - agentState.pose.position.y),
    //                                (state_r2->values[0] - agentState.pose.position.x));

    double tethaRobotAgent = atan2((odomData->pose.pose.position.y - agentState.pose.position.y),
                                   (odomData->pose.pose.position.x - agentState.pose.position.x));

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
            modSigmaY = (1 + agentVelocity * fv + fFront + fFieldOfView) * sigmaY;
        else
            modSigmaY = (1 + agentVelocity * fv + fFront) * sigmaY;
    }
    else
    {
        modSigmaY = sigmaY;
    }

    double basicPersonalSpaceVal =
        Ap *
        std::exp(-(
            std::pow(dRobotAgent * std::cos(tethaRobotAgent - tethaOrientation) / (std::sqrt(2) * sigmaX),
                     2) +
            std::pow(dRobotAgent * std::cos(tethaRobotAgent - tethaOrientation) / (std::sqrt(2) * modSigmaY),
                     2)));

    return basicPersonalSpaceVal;
}

bool OmFclStateValidityCheckerR2::isRobotInFront(const ob::State *state,
                                                 const pedsim_msgs::AgentState agentState,
                                                 const ob::SpaceInformationPtr space) const

{
    double tethaAgentRobot = atan2((odomData->pose.pose.position.y - agentState.pose.position.y),
                                   (odomData->pose.pose.position.x - agentState.pose.position.x));

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

bool OmFclStateValidityCheckerR2::isAgentInRFOV(const ob::State *state,
                                                const pedsim_msgs::AgentState agentState,
                                                const ob::SpaceInformationPtr space) const

{
    // ROS_INFO_STREAM("running agent fov fnc");

    // geometry_msgs::PoseStamped robotPose;
    // geometry_msgs::PoseStamped agentPose;

    // robotPose.pose = odomData->pose.pose;
    // agentPose.pose = agentState.pose;

    // ROS_INFO_STREAM("First agent pose x: " << agentPose.pose.position.x);
    // ROS_INFO_STREAM("First agent pose y: " << agentPose.pose.position.y);

    // tf::TransformListener tl;
    // tl.transformPose(main_frame, robotPose, agentPose);

    // const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    // ob::ScopedState<> agentTf(space);
    // agentTf[0] = double(agentState.pose.position.x);  // x
    // agentTf[1] = double(agentState.pose.position.y);  // y

    // double dRobotAgent = space->distance(state, agentTf->as<ob::State>());

    //  dRobotAgent -

    double dRobotAgent = std::sqrt(std::pow(agentState.pose.position.x - odomData->pose.pose.position.x, 2) +
                                   std::pow(agentState.pose.position.y - odomData->pose.pose.position.y, 2));

    double robotVelocity =
        std::sqrt(std::pow(odomData->twist.twist.linear.x, 2) + std::pow(odomData->twist.twist.linear.y, 2));

    double actualFOVDistance = robotDistanceView / robotVelocityThreshold * robotVelocity;

    if (actualFOVDistance < 1.5)
    {
        actualFOVDistance = 1.5;
    }

    // ROS_INFO_STREAM("distance of robot view " << actualFOVDistance);

    if (dRobotAgent > actualFOVDistance)
    {
        return false;
    }

    // ROS_INFO_STREAM("Agents in radius");

    double tethaRobotAgent = atan2((agentState.pose.position.y - odomData->pose.pose.position.y),
                                   (agentState.pose.position.x - odomData->pose.pose.position.x));

    if (tethaRobotAgent < 0)
    {
        tethaRobotAgent = 2 * M_PI + tethaRobotAgent;
    }

    // ROS_INFO_STREAM("Angle robot agent: " << tethaRobotAgent);

    tf::Quaternion q(odomData->pose.pose.orientation.x, odomData->pose.pose.orientation.y,
                     odomData->pose.pose.orientation.z, odomData->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double robotAngle = yaw;

    if (robotAngle < 0)
    {
        robotAngle = 2 * M_PI + robotAngle;
    }

    // ROS_INFO_STREAM("robot pure angle: " << robotAngle);

    // if (tethaRobotAgent < robotAngle)
    //     tethaRobotAgent = robotAngle - tethaRobotAgent;
    // else if (tethaRobotAgent > robotAngle)
    //     tethaRobotAgent = tethaRobotAgent - robotAngle;
    // else
    //     return true;

    if (tethaRobotAgent > (robotAngle + M_PI))
        tethaRobotAgent = abs(robotAngle + 2 * M_PI - tethaRobotAgent);
    else if (robotAngle > (tethaRobotAgent + M_PI))
        tethaRobotAgent = abs(tethaRobotAgent + 2 * M_PI - robotAngle);
    else
        tethaRobotAgent = abs(tethaRobotAgent - robotAngle);

    // ROS_INFO_STREAM("diff angle: " << tethaRobotAgent);
    // ROS_INFO_STREAM("perm angle: " << fRobotView);

    if (abs(tethaRobotAgent) < fRobotView)
        return true;

    return false;
}

bool OmFclStateValidityCheckerR2::isValidPoint(const ob::State *state) const
{
    OcTreeNode *result;
    point3d query;
    double node_occupancy;

    // extract the component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    query.x() = state_r2->values[0];
    query.y() = state_r2->values[1];
    query.z() = 0.0;

    result = octree_->search(query);

    if (result == NULL)
    {
        return false;
    }
    else
    {
        node_occupancy = result->getOccupancy();
        if (node_occupancy <= 0.4)
            return true;
    }
    return false;
}

OmFclStateValidityCheckerR2::~OmFclStateValidityCheckerR2()
{
    delete octree_;
    //    delete tree_;
    //    delete tree_obj_;
}
