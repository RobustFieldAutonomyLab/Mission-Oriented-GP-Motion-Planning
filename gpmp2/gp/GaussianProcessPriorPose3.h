#pragma once

#include "../gp/GaussianProcessPriorLie.h"

#include <gtsam/geometry/Pose3.h>

namespace gpmp2 {

    typedef GaussianProcessPriorLie<gtsam::Pose3> GaussianProcessPriorPose3;

} // \ namespace gpmp2
