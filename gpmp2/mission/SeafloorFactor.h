#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <iostream>
#include <vector>

#include "Seafloor.h"


namespace gpmp2{


template <class ROBOT>
class SeafloorFactor: public gtsam::NoiseModelFactor1<typename ROBOT::Pose> {

    public:
        // typedefs
        typedef ROBOT Robot;
        typedef typename Robot::Pose Pose;

    private:
        // typedefs
        typedef SeafloorFactor This;
        typedef gtsam::NoiseModelFactor1<Pose> Base;

        // seafloor cost settings
        double epsilon_{};      // distance from seafloor that start non-zero cost

        // arm: planar one, all alpha = 0
        const Robot& robot_;

        // signed distance field from matlab
        const Seafloor& sf_;


    public:

        /// shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<This> shared_ptr;

        /* Default constructor do nothing */
        SeafloorFactor() : robot_(Robot()) {}

        /**
         * Constructor
         * @param cost_model cost function covariance, should to identity model
         * @param field      signed distance field
         * @param nn_index   nearest neighbour index of signed distance field
         */
        SeafloorFactor(gtsam::Key poseKey, const Robot& robot,
                          const Seafloor& sf, double cost_sigma, double epsilon) :
                Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(), cost_sigma), poseKey),
                epsilon_(epsilon), robot_(robot), sf_(sf) {}

        virtual ~SeafloorFactor() = default;


        /// error function
        /// numerical jacobians / analytic jacobians from cost function
        gtsam::Vector evaluateError(const typename Robot::Pose& conf,
                                    boost::optional<gtsam::Matrix&> H1 = boost::none) const ;


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

#include "SeafloorFactor-inl.h"