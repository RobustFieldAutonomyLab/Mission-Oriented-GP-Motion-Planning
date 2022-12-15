#pragma once

#include "../kinematics/Pose3MobileBaseModel.h"
#include "../gp/GaussianProcessInterpolatorPose3.h"
#include "../obstacle/ObstacleSDFFactorGP.h"

namespace gpmp2 {

// template uses PointRobotModel as robot type
    typedef ObstacleSDFFactorGP<Pose3MobileBaseModel, GaussianProcessInterpolatorPose3>
            ObstacleSDFFactorGPPose3MobileBase;

}