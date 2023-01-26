
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
        double dist, dist_symbol;
        try {
            dist = sf.getDistance(point);
        } catch (SDFQueryOutOfRange&) {
            if (H_point) *H_point = gtsam::Matrix13::Zero();
            return 0.0;
        }

        if (dist < 0.0) {
            //the robot is below seafloor, not allowed
            dist = -dist * 10;
            dist_symbol = -10;
//            dist = 1000;
//            dist_symbol = 1000;
        }
        else dist_symbol = 1;

        if (dist < eps && dist_symbol == 1) {
            // close enough no error
            if (H_point) *H_point = gtsam::Matrix13::Zero();
                return 0.0;

        } else {
            // far away, give far away punishment

            if (H_point) {
                double derr = 1*exp(-dist) / (1+exp(-dist)) / (1+exp(-dist));
                //TODO:fix this
                *H_point = (gtsam::Matrix13() << 0, 0, derr * dist_symbol).finished();
            }
            return 1/(1 + exp(-dist));
        }
    }

}