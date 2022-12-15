/**
 *  @file   Pose2MobileBaseModel.h
 *  @brief  SE(2) mobile base with physical body, which is represented by spheres
 *  @author Mustafa Mukadam
 *  @date   Jan 22, 2018
 **/

#pragma once

#include "Pose2MobileBase.h"
#include "RobotModel.h"

namespace gpmp2 {

/**
 * SE(2) mobile base with physical body, which is represented by spheres
 * Used to check collisions
 */
typedef RobotModel<Pose2MobileBase> Pose2MobileBaseModel;

}
