#pragma once

#include "../kinematics/Pose3MobileBaseModel.h"
#include "../obstacle/ObstacleSDFFactor.h"

namespace gpmp2 {

// template use BaseModel as robot type
    typedef ObstacleSDFFactor<Pose3MobileBaseModel> ObstacleSDFFactorPose3MobileBase;

}

