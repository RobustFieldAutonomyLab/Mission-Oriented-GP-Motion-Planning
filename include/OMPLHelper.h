#pragma once

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <ompl/config.h>

#include <boost/filesystem.hpp>
#include <iostream>

#include "../gpmp2/obstacle/SignedDistanceField.h"

#include "../gpmp2/mission/Seafloor.h"
#include "gtsam/geometry/Pose3.h"

enum OMPLMethod{RRTStar, LBKPiece};

struct OMPLParam{
    OMPLMethod method_;

    gtsam::Point3 origin_;
    double cell_size_;
    double cell_size_z_;

    double vehicle_size_;
    double dist_sdf_;
    double dist_sf_;

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

inline gtsam::Pose3 SE3StateSpace2Pose3(const ompl::base::State* s){
    namespace ob = ompl::base;
    double x = s->as<ob::SE3StateSpace::StateType>()->getX();
    double y = s->as<ob::SE3StateSpace::StateType>()->getY();
    double z = s->as<ob::SE3StateSpace::StateType>()->getZ();
    double qx = s->as<ob::SE3StateSpace::StateType>()->rotation().x;
    double qy = s->as<ob::SE3StateSpace::StateType>()->rotation().y;
    double qz = s->as<ob::SE3StateSpace::StateType>()->rotation().z;
    double qw = s->as<ob::SE3StateSpace::StateType>()->rotation().w;
    return gtsam::Pose3(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
}

class vehicleDynamicsObjective : public ompl::base::StateCostIntegralObjective
{
public:
    explicit vehicleDynamicsObjective(const ompl::base::SpaceInformationPtr& si) :
            ompl::base::StateCostIntegralObjective(si, true)
    {
    }

    ompl::base::Cost motionCost(const ompl::base::State* s1, const ompl::base::State* s2) const override
    {
        namespace ob = ompl::base;
        auto pose1 = SE3StateSpace2Pose3(s1);
        auto pose2 =SE3StateSpace2Pose3(s2);
        auto pose_btwn = pose1.transform_pose_to(pose2);
        double err = pose_btwn.y()*pose_btwn.y() +
                pose_btwn.rotation().pitch() * pose_btwn.rotation().pitch() * 100;

        return ompl::base::Cost(err);
    }
};

class seafloorFollowingObjective : public ompl::base::StateCostIntegralObjective
{
public:
    explicit seafloorFollowingObjective(const ompl::base::SpaceInformationPtr& si,
                                        const gpmp2::Seafloor& sf, double safe_dist) :
            ompl::base::StateCostIntegralObjective(si, true),
            sf_(sf), safe_dist_(safe_dist)
    {
    }

    ompl::base::Cost stateCost(const ompl::base::State* s) const override
    {
        namespace ob = ompl::base;
        auto p_tmp = SE3StateSpace2Pose3(s);
        double dist = sf_.getDistance(p_tmp.translation());
        double err;
        if(dist < safe_dist_){
            err = 0;
        }
        else
            err = 1/(1 + exp(-dist));
        return ompl::base::Cost(err);
    }

private:
    gpmp2::Seafloor sf_;
    double safe_dist_;
};

class signedDistanceFieldObjective : public ompl::base::StateCostIntegralObjective
{
public:
    explicit signedDistanceFieldObjective(const ompl::base::SpaceInformationPtr& si,
                          const gpmp2::SignedDistanceField& sdf, double safe_dist) :
            ompl::base::StateCostIntegralObjective(si, true),
            sdf_(sdf), safe_dist_(safe_dist)
    {
    }

    ompl::base::Cost stateCost(const ompl::base::State* s) const override
    {
        namespace ob = ompl::base;
        auto p_tmp = SE3StateSpace2Pose3(s);
        double dist = sdf_.getSignedDistance(p_tmp.translation());
        double err;
        if(dist > safe_dist_){
            err = 0;
        }
        else
            err = safe_dist_ - dist;
        return ompl::base::Cost(err);
    }

private:
    gpmp2::SignedDistanceField sdf_;
    double safe_dist_;
};