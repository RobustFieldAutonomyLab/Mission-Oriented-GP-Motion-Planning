#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <iostream>
#include <vector>

#include "WaterCurrentGrid.h"


namespace gpmp2{

/**
* binary factor for sealevel flowing use GP interpolation, planar version
* template robot model and GP interpolator version
*/
    template <class ROBOT, class GPINTER>
    class SealevelFactorGP: public gtsam::NoiseModelFactor4<
            typename ROBOT::Pose, typename ROBOT::Velocity,
            typename ROBOT::Pose, typename ROBOT::Velocity> {

    public:
        // typedefs
        typedef ROBOT Robot;
        typedef typename Robot::Pose Pose;
        typedef typename Robot::Velocity Velocity;

    private:
        // typedefs
        typedef SealevelFactorGP This;
        typedef gtsam::NoiseModelFactor4<Pose, Velocity, Pose, Velocity> Base;
        typedef GPINTER GPBase;

        // GP interpolator
        GPBase GPbase_;

        // obstacle settings
        double epsilon_;      // global eps_ for hinge loss function

        // physical arm, with body sphere information
        const Robot& robot_;

        double sea_level_;

        const WaterCurrentGrid& wcg_;

        bool use_current_;


    public:

        /// shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<This> shared_ptr;

        /* Default constructor do nothing */
        SealevelFactorGP() : robot_(Robot()),
                             wcg_(WaterCurrentGrid()) {}

        /**
         * Constructor
         * @param cost_model cost function covariance, should to identity model
         * @param Qc_model   dim is equal to DOF
         * @param field      signed distance field
         * @param check_inter  how many points needed to be interpolated. 0 means no GP interpolation
         */
        SealevelFactorGP(
                gtsam::Key pose1Key, gtsam::Key vel1Key, gtsam::Key pose2Key, gtsam::Key vel2Key,
                const Robot& robot, const double sea_level, double cost_sigma,
                double epsilon, const gtsam::SharedNoiseModel& Qc_model, double delta_t, double tau) :

                Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(), cost_sigma),
                     pose1Key, vel1Key, pose2Key, vel2Key), GPbase_(Qc_model, delta_t, tau),epsilon_(epsilon),
                robot_(robot), sea_level_(sea_level), wcg_(WaterCurrentGrid()), use_current_(false)  {}

        SealevelFactorGP(
                gtsam::Key pose1Key, gtsam::Key vel1Key, gtsam::Key pose2Key, gtsam::Key vel2Key,
                const Robot& robot, const double sea_level, const WaterCurrentGrid& wcg, double cost_sigma,
                double epsilon, const gtsam::SharedNoiseModel& Qc_model, double delta_t, double tau) :

                Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(), cost_sigma),
                     pose1Key, vel1Key, pose2Key, vel2Key), GPbase_(Qc_model, delta_t, tau),
                epsilon_(epsilon), robot_(robot), sea_level_(sea_level), wcg_(wcg), use_current_(true) {}


        virtual ~SealevelFactorGP() {}


        /// error function
        /// numerical jacobians / analytic jacobians from cost function
        gtsam::Vector evaluateError(
                const typename Robot::Pose& conf1, const typename Robot::Velocity& vel1,
                const typename Robot::Pose& conf2, const typename Robot::Velocity& vel2,
                boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none,
                boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none) const ;


        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

        /** print contents */
        void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter =gtsam:: DefaultKeyFormatter) const {
            std::cout << s << "SealevelFactorGP :" << std::endl;
            Base::print("", keyFormatter);
        }


        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int version) {
            ar & boost::serialization::make_nvp("NoiseModelFactor4",
                                                boost::serialization::base_object<Base>(*this));
        }


    };

}

#include "SealevelFactorGP-inl.h"