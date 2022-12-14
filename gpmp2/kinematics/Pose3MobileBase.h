//
// Created by rfal on 12/5/22.
//

#pragma once

#ifndef GPMP_STR_POSE3MOBILEBASE_H
#define GPMP_STR_POSE3MOBILEBASE_H

#include "../kinematics/ForwardKinematics.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>

#include <vector>

namespace gpmp2{
    class Pose3MobileBase : public ForwardKinematics<gtsam::Pose3, gtsam::Vector>{

    private:
        typedef ForwardKinematics<gtsam::Pose3, gtsam::Vector> Base;

    public:
        /// default constructor
        explicit Pose3MobileBase() : Base(6,1) {}

        /// default constructor
        virtual ~Pose3MobileBase() {}


        /**
         *  Forward kinematics: joint configuration to poses in workspace
         *  Velocity kinematics: optional joint velocities to linear velocities in workspace, no anuglar rate
         *
         *  @param p position in config space
         *  @param v velocity in config space
         *  @param px link pose in work space
         *  @param vx link velocity in work space
         *  @param J_px_p et al. optional Jacobians
         **/
        void forwardKinematics(const gtsam::Pose3& p, boost::optional<const gtsam::Vector&> v,
                               std::vector<gtsam::Pose3>& px, boost::optional<std::vector<gtsam::Vector6>&> vx,
                               boost::optional<std::vector<gtsam::Matrix>&> J_px_p = boost::none,
                               boost::optional<std::vector<gtsam::Matrix>&> J_vx_p = boost::none,
                               boost::optional<std::vector<gtsam::Matrix>&> J_vx_v = boost::none) const;
    };

}




#endif //GPMP_STR_POSE3MOBILEBASE_H
