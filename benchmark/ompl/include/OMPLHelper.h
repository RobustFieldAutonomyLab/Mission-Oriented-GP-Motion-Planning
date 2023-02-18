#pragma once

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
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


//enum OMPLMethod{RRTStar, LBKPiece, PRM};

struct OMPLParameter{
    std::string method;
    double vehicle_size;

    double dist_sdf;
    double w_sdf;

    bool seafloor_mission;
    double dist_sf;
    double w_sf;
    double sea_level;

    double w_vd;

    gtsam::Point3 origin;
    double cell_size;
    double cell_size_z;

    double cost_threshold;
    double max_time;
};

class OMPLHelper
{
public:
    OMPLHelper(const std::string& ppm_file, OMPLParameter params);

    bool plan(gtsam::Pose3 start_pt, gtsam::Pose3 end_pt);

    void recordSolution(PLOT_TYPE tp, std::string file_name);

private:
    bool isStateValid(const ompl::base::State *state) const;

    ompl::geometric::SimpleSetupPtr ss_;

    std::string method_;
    double max_time_;

    gtsam::Point3 origin_, corner_;

    double cell_size_;
    double vehicle_size_;

    gpmp2::Seafloor* sf_;
    double dist_sf_;

    gpmp2::SignedDistanceField* sdf_;
    double dist_sdf_;

};
