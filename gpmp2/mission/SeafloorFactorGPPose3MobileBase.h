#pragma once

#include "../kinematics/Pose3MobileBaseModel.h"
#include "../gp/GaussianProcessInterpolatorPose3.h"
#include "SeafloorFactorGP.h"

namespace gpmp2 {

// template use BaseModel as robot type
    typedef SeafloorFactorGP<Pose3MobileBaseModel, GaussianProcessInterpolatorPose3> SeafloorFactorGPPose3MobileBase;

}