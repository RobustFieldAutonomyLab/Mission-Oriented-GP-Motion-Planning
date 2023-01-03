
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include "Seafloor.h"

namespace gpmp2 {
    /// seafloor cost function
    inline double SeafloorCost(const gtsam::Point3& point, const Seafloor& sf,
                                        double eps, gtsam::OptionalJacobian<1, 3> H_point = boost::none) {
        double dist;
        try {
            dist = sf.getDistance(point);
        } catch (SDFQueryOutOfRange&) {
            if (H_point) *H_point = gtsam::Matrix13::Zero();
            return 0.0;
        }

        if (dist < eps) {
            // close enough no error
            if (H_point) *H_point = gtsam::Matrix13::Zero();
            return 0.0;

        } else {
            // far away, give far away punishment
            if (H_point) *H_point = (gtsam::Matrix13() << 0, 0, 1).finished();
            return -dist;
        }
    }

}