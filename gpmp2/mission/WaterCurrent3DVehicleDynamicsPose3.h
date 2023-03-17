#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include "WaterCurrentGrid.h"

namespace gpmp2{
    inline double WaterCurrent3DVehicleDynamicsPose3(const gtsam::Pose3& p, const gtsam::Vector6& v,
                                        const WaterCurrentGrid& wcg,
                                        gtsam::OptionalJacobian<1, 6> Hp = boost::none,
                                        gtsam::OptionalJacobian<1, 6> Hv = boost::none) {
        gtsam::Vector6 real_v = wcg.getVehicleVelocityWithoutCurrent(p.translation(), v);

        double err = v(4);
        if (Hp) *Hp = (gtsam::Matrix16() << 0, 0, 0, 0, 0, 0).finished();
        if (Hv) *Hv = (gtsam::Matrix16() << 0, 0, 0, 0, 1, 0).finished();

        //TODO: add a scale factor for the angle
        return err;
    }
}
