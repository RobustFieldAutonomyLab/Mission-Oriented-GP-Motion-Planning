/**
 *  @file   ObstaclePlanarSDFFactorGPPose2MobileBase.h
 *  @brief  Obstacle avoidance interpolated GP factor for 2D SE(2) base
 *  @author Mustafa Mukadam
 *  @date   Jan 23, 2018
 **/

#pragma once

#include "../kinematics/Pose2MobileBaseModel.h"
#include "../gp/GaussianProcessInterpolatorPose2.h"
#include "../obstacle/ObstaclePlanarSDFFactorGP.h"

namespace gpmp2 {

// template uses PointRobotModel as robot type
typedef ObstaclePlanarSDFFactorGP<Pose2MobileBaseModel, GaussianProcessInterpolatorPose2>
    ObstaclePlanarSDFFactorGPPose2MobileBase;

}
