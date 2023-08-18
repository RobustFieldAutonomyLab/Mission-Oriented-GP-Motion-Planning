#pragma once

#include "../kinematics/Pose3MobileBaseModel.h"
#include "../gp/GaussianProcessInterpolatorPose3.h"
#include "SealevelFactorGP.h"

namespace gpmp2 {

// template use BaseModel as robot type
    typedef SealevelFactorGP<Pose3MobileBaseModel, GaussianProcessInterpolatorPose3> SealevelFactorGPPose3MobileBase;

}