//
// Created by rfal on 12/14/22.
//

#ifndef GPMP_STR_GAUSSIANPROCESSINTERPOLATORPOSE3_H
#define GPMP_STR_GAUSSIANPROCESSINTERPOLATORPOSE3_H

#pragma once

#include "../gp/GaussianProcessInterpolatorLie.h"

#include <gtsam/geometry/Pose3.h>

namespace gpmp2 {

    typedef GaussianProcessInterpolatorLie<gtsam::Pose3> GaussianProcessInterpolatorPose3;

} // \ namespace gpmp2


#endif //GPMP_STR_GAUSSIANPROCESSINTERPOLATORPOSE3_H
