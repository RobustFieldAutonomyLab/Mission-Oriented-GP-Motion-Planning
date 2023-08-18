#pragma once

#include "../kinematics/Pose3MobileBaseModel.h"
#include "SealevelFactor.h"

namespace gpmp2 {
    typedef SealevelFactor<Pose3MobileBaseModel> SealevelFactorPose3MobileBase;

}