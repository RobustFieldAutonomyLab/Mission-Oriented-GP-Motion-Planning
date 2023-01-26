#pragma once

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/GeneticSearch.h>


#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/config.h>

#include <boost/filesystem.hpp>
#include <iostream>

#include "OMPLConstraint.h"
#include "OMPLObjective.h"

#include "../include/Visualization.h"
#include "../include/SignedDistanceField.h"


enum OMPLMethod{RRTStar, LBKPiece, GeneticAlgorithm};

struct OMPLParam{
    OMPLMethod method_;

    gtsam::Point3 origin_;
    double cell_size_;
    double cell_size_z_;

    double vehicle_size_;
    double dist_sdf_;
    double dist_sf_;

    double w_sf_;
    double w_sdf_;
    double w_vd_;

    double cost_thres_;

    bool dynamic_constraint;

};

class OMPLHelper
{
public:
    OMPLHelper(const char *ppm_file, OMPLParam params);

    bool plan(gtsam::Pose3 start_pt, gtsam::Pose3 end_pt);

    void recordSolution();

private:
    bool isStateValid(const ompl::base::State *state) const;

    ompl::geometric::SimpleSetupPtr ss_;

    OMPLMethod method_;

    gtsam::Point3 origin_, corner_;

    double cell_size_;
    double vehicle_size_;

    gpmp2::Seafloor* sf_;
    double dist_sf_;

    gpmp2::SignedDistanceField* sdf_;
    double dist_sdf_;

};
