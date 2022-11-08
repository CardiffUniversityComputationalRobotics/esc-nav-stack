/*! \file state_cost_objective.hpp
 * \brief State cost objective.
 *
 * \date Mar 31, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
 *
 * \details State cost objective. Define different state cost objectives.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#ifndef OMPL_CONTRIB_STATE_COST_OBJECTIVES_
#define OMPL_CONTRIB_STATE_COST_OBJECTIVES_

// OMPL
#include <ompl/config.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>

#include <state_validity_checker_grid_map_R2.h>

namespace ob = ompl::base;

class PathLengthGoalRegionOptimizationObjective : public ob::OptimizationObjective
{
public:
    ob::State *goal_;
    double goal_radius_;
    PathLengthGoalRegionOptimizationObjective(const ob::SpaceInformationPtr &si, const ob::State *goal,
                                              const double &goal_radius)
        : ob::OptimizationObjective(si)
    {
        description_ = "Path Length Goal Region";

        // Setup a default cost-to-go heuristics:
        setCostToGoHeuristic(ob::goalRegionCostToGo);
        goal_radius_ = goal_radius;
        goal_ = si->cloneState(goal);
    }

    /** \brief Returns identity cost. */
    ob::Cost stateCost(const ob::State *s) const
    {
        // ROS_INFO_STREAM("running path length");
        return identityCost();
    }

    /** \brief Motion cost for this objective is defined as
        the configuration space distance between \e s1 and \e
        s2, using the method SpaceInformation::distance(). */
    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const
    {
        double s2_distance_to_goal = si_->distance(s2, goal_);
        if (s2_distance_to_goal < goal_radius_)
            return ob::Cost(s2_distance_to_goal + si_->distance(s1, s2));
        else
            return ob::Cost(si_->distance(s1, s2));
    }

    /** \brief the motion cost heuristic for this objective is
        simply the configuration space distance between \e s1
        and \e s2, since this is the optimal cost between any
        two states assuming no obstacles. */
    ob::Cost motionCostHeuristic(const ob::State *s1, const ob::State *s2) const
    {
        return motionCost(s1, s2);
    }

    /** \brief Allocate a state sampler for the path-length objective (i.e., direct ellipsoidal sampling). */
    ob::InformedSamplerPtr allocInformedStateSampler(const ob::ProblemDefinitionPtr &probDefn,
                                                     unsigned int maxNumberCalls) const
    {
        // Make the direct path-length informed sampler and return. If OMPL was compiled with Eigen, a direct
        // version is available, if not a rejection-based technique can be used
        return std::make_shared<ob::PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
    }
};

class ExtendedSocialComfortObjective : public ob::StateCostIntegralObjective
{
private:
public:
    ExtendedSocialComfortObjective(const ob::SpaceInformationPtr &si, bool enableMotionCostInterpolation)
        : ob::StateCostIntegralObjective(si, enableMotionCostInterpolation)
    {
        // ROS_INFO_STREAM("running extended social comfort objective checker");
    }

    // in case of modifying the cost calculation function here it is where it should be done

    ob::Cost stateCost(const ob::State *s) const
    {
        std::shared_ptr<GridMapStateValidityCheckerR2> state_vality_checker =
            std::static_pointer_cast<GridMapStateValidityCheckerR2>(si_->getStateValidityChecker());
        return ob::Cost(state_vality_checker->checkExtendedSocialComfort(s, si_));
    }

    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const
    {
        if (interpolateMotionCost_)
        {
            ob::Cost totalCost = this->identityCost();

            int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
            // nd = int(nd/10);

            ob::State *test1 = si_->cloneState(s1);
            ob::Cost prevStateCost = this->stateCost(test1);
            if (nd > 1)
            {
                ob::State *test2 = si_->allocState();
                for (int j = 1; j < nd; ++j)
                {
                    si_->getStateSpace()->interpolate(s1, s2, (double)j / (double)nd, test2);
                    ob::Cost nextStateCost = this->stateCost(test2);
                    totalCost = ob::Cost(
                        totalCost.value() +
                        this->trapezoid(prevStateCost, nextStateCost, si_->distance(test1, test2)).value());
                    std::swap(test1, test2);
                    prevStateCost = nextStateCost;
                }
                si_->freeState(test2);
            }

            // Lastly, add s2
            totalCost = ob::Cost(
                totalCost.value() +
                this->trapezoid(prevStateCost, this->stateCost(s2), si_->distance(test1, s2)).value());

            si_->freeState(test1);

            // ROS_INFO_STREAM("Total cost: " << totalCost);

            return totalCost;
        }
        else
        {
            // ROS_INFO_STREAM("Trapezoid cost: " << this->trapezoid(this->stateCost(s1), this->stateCost(s2),
            //                                                       si_->distance(s1, s2)));
            return this->trapezoid(this->stateCost(s1), this->stateCost(s2), si_->distance(s1, s2));
        }
    }
};

/** Return an optimization objective which attempts to steer the robot
    away from social agents in order to keep comfort with an extended social comfort model. */
ob::OptimizationObjectivePtr getExtendedSocialComfortObjective(const ob::SpaceInformationPtr &si,
                                                               bool motion_cost_interpolation);

/** Returns a structure representing the optimization objective to use
    for optimal motion planning. This method returns an objective
    which attempts to minimize the length in configuration space of
    computed paths. */
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr &si);

ob::OptimizationObjectivePtr getPathLengthGoalRegionObjective(const ob::SpaceInformationPtr &si,
                                                              const ob::State *goal,
                                                              const double &goal_radius);
#endif
