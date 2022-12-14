#pragma once

#include "../kinematics/Pose3MobileBase.h"
#include "../kinematics/RobotModel.h"

namespace gpmp2 {

/**
 * SE(2) mobile base with physical body, which is represented by spheres
 * Used to check collisions
 */
    typedef RobotModel<Pose3MobileBase> Pose3MobileBaseModel;

}