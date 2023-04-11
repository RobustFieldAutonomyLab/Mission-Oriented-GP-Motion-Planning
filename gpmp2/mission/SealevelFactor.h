#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <iostream>
#include <vector>

#include "SeafloorCost.h"

namespace gpmp2{
    template <class ROBOT>
    class SealevelFactor: public gtsam::NoiseModelFactor1<typename ROBOT::Pose> {
    public:
        // typedefs
        typedef ROBOT Robot;
        typedef typename Robot::Pose Pose;

    private:
        // typedefs
        typedef SealevelFactor This;
        typedef gtsam::NoiseModelFactor1<Pose> Base;

        // seafloor cost settings
        double epsilon_{};      // distance from seafloor that start non-zero cost
        double sea_level_{};    // sea level

        // arm: planar one, all alpha = 0
        const Robot& robot_;

    public:

        /// shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<This> shared_ptr;

        /* Default constructor do nothing */
        SealevelFactor() : robot_(Robot()) {}

        /**
         * Constructor
         * @param cost_model cost function covariance, should to identity model
         * @param field      signed distance field
         * @param nn_index   nearest neighbour index of signed distance field
         */
        SealevelFactor(gtsam::Key poseKey, const Robot& robot, const double sea_level,
         double cost_sigma, double epsilon) :
        Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(), cost_sigma), poseKey),
        epsilon_(epsilon), robot_(robot), sea_level_(sea_level){}

        virtual ~SealevelFactor() = default;


        /// error function
        /// numerical jacobians / analytic jacobians from cost function
        gtsam::Vector evaluateError(const typename Robot::Pose& conf,
                                    boost::optional<gtsam::Matrix&> H1 = boost::none) const {
            // if Jacobians used, initialize as zeros
            // size: arm_nr_points_ * DOF
            if (H1) *H1 = gtsam::Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());

            // run forward kinematics of this configuration
            std::vector<gtsam::Point3> sph_centers;
            std::vector<gtsam::Matrix> J_px_jp;
            if (H1)
                robot_.sphereCenters(conf, sph_centers, J_px_jp);
            else
                robot_.sphereCenters(conf, sph_centers);


            // allocate cost vector
            gtsam::Vector err(robot_.nr_body_spheres());

            // for each point on arm stick, get error
            for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {

                const double total_eps = robot_.sphere_radius(sph_idx) + epsilon_;

                if (H1) {
                    gtsam::Matrix13 Jerr_point;
                    err(sph_idx) = SealevelCost(sph_centers[sph_idx], sea_level_, total_eps, Jerr_point);

                    // chain rules
                    H1->row(sph_idx) = Jerr_point * J_px_jp[sph_idx];
//            cout<< "H1"<<*H1<<endl;
                } else {
                    err(sph_idx) = SealevelCost(sph_centers[sph_idx], sea_level_, total_eps);
                }
            }
//    cout<<"err"<<err<<endl;

            return err;
        }


        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

        /** print contents */
        void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
            std::cout << s << "SeafloorFactor :" << std::endl;
            Base::print("", keyFormatter);
        }


        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int version) {
            ar & boost::serialization::make_nvp("NoiseModelFactor1",
                                                boost::serialization::base_object<Base>(*this));
        }
    };
}