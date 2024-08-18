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
                                                             std::vector<double> planning_bounds_y, grid_map::GridMap grid_map, const double robot_radius, std::shared_ptr<octomap::OcTree> octree)
    : ob::StateValidityChecker(si), robot_base_radius_(0.4)
{

    opport_collision_check_ = opport_collision_check;
    planning_bounds_x_ = planning_bounds_x;
    planning_bounds_y_ = planning_bounds_y;
    robot_base_radius_ = robot_radius;
    robot_base_height_ = 1.5;

    grid_map_ = grid_map;

    try
    {
        obstacles_grid_map_ = grid_map_.get("full");
        comfort_grid_map_ = grid_map_.get("comfort");

        // OCTOMAP PROCESS
        octree_ = octree;
        tree_ = new fcl::OcTreef(octree_);
        tree_obj_ = new fcl::CollisionObjectf((std::shared_ptr<fcl::CollisionGeometryf>(tree_)));

        robot_collision_solid_.reset(new fcl::Cylinderf(robot_base_radius_, robot_base_height_));

        octree_res_ = octree->getResolution();
        octree_->getMetricMin(octree_min_x_, octree_min_y_, octree_min_z_);
        octree_->getMetricMax(octree_max_x_, octree_max_y_, octree_max_z_);
    }
    catch (...)
    {
    }
}

bool GridMapStateValidityCheckerR2::isValid(const ob::State *state) const
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
    fcl::Transform3f robot_tf;
    robot_tf.setIdentity();
    robot_tf.translate(fcl::Vector3f(state_r2->values[0], state_r2->values[1], robot_base_height_ / 2.0));

    fcl::CollisionObjectf vehicle_co(robot_collision_solid_, robot_tf);

    fcl::CollisionRequestf collision_request;
    fcl::CollisionResultf collision_result;

    fcl::collide(tree_obj_, &vehicle_co, collision_request, collision_result);

    // std::cout << "Collision (FCL): " << collision_result.isCollision() << std::endl;

    if (collision_result.isCollision())
    {
        // ompl::tools::Profiler::End("collision");
        return false;
    }

    return true;
}

double GridMapStateValidityCheckerR2::checkExtendedSocialComfort(const ob::State *state,
                                                                 const ob::SpaceInformationPtr space) const
{
    double state_risk = 0.0;

    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    grid_map::Position query(state_r2->values[0], state_r2->values[1]);

    grid_map::Index index;

    if (grid_map_.getIndex(query, index))
    {
        state_risk = comfort_grid_map_(index(0), index(1));
    }

    if (state_risk < 1 || isnan(state_risk))
    {
        state_risk = 1;
    }

    return state_risk;
}

bool GridMapStateValidityCheckerR2::isValidPoint(const ob::State *state) const
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
        if (node_occupancy <= 0.2)
            return true;
    }
    return false;
}

GridMapStateValidityCheckerR2::~GridMapStateValidityCheckerR2()
{
}
