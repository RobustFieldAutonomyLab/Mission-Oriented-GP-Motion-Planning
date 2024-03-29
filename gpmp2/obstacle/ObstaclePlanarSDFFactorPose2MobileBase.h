/**
 *  @file   ObstaclePlanarSDFFactorPose2MobileBase.h
 *  @brief  Obstacle avoidance cost factor for 2D SE(2) base
 *  @author Mustafa Mukadam
 *  @date   Jan 23, 2018
 **/

#pragma once

#include "../kinematics/Pose2MobileBaseModel.h"
#include "../obstacle/ObstaclePlanarSDFFactor.h"

namespace gpmp2 {

// template use BaseModel as robot type
typedef ObstaclePlanarSDFFactor<Pose2MobileBaseModel> ObstaclePlanarSDFFactorPose2MobileBase;

}


