#pragma once

#include "../kinematics/Pose3MobileBaseModel.h"
#include "SeafloorFactor.h"

namespace gpmp2 {

// template use BaseModel as robot type
    typedef SeafloorFactor<Pose3MobileBaseModel> SeafloorFactorPose3MobileBase;

}