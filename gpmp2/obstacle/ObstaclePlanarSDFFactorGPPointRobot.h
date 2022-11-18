/**
 *  @file   ObstaclePlanarSDFFactorGPPointRobot.h
 *  @brief  Obstacle avoidance cost factor, for point robot, using signed distance field,
 *          and GP interpolation
 *  @author Mustafa Mukadam
 *  @date   July 20, 2016
 **/

#pragma once

#include "../kinematics/PointRobotModel.h"
#include "../gp/GaussianProcessInterpolatorLinear.h"
#include "ObstaclePlanarSDFFactorGP.h"

namespace gpmp2 {

// template uses PointRobotModel as robot type
typedef ObstaclePlanarSDFFactorGP<PointRobotModel, GaussianProcessInterpolatorLinear>
    ObstaclePlanarSDFFactorGPPointRobot;

}
