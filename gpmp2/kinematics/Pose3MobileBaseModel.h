#pragma once

#include "Pose3MobileBase.h"
#include "RobotModel.h"

namespace gpmp2 {

/**
 * SE(3) mobile base with physical body, which is represented by spheres
 * Used to check collisions
 */
    typedef RobotModel<Pose3MobileBase> Pose3MobileBaseModel;

}