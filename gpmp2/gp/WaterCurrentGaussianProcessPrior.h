#pragma once

#include "../mission/WaterCurrentGrid.h"

#include "../gp/GPutils.h"

#include <gtsam/geometry/concepts.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>

#include <boost/lexical_cast.hpp>
#include <boost/serialization/export.hpp>

#include <ostream>


namespace gpmp2 {

/**
 * 4-way factor for Gaussian Process prior factor on any Lie group
 */
class WaterCurrentGaussianProcessPrior: public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector,
            gtsam::Pose3, gtsam::Vector> {

    private:
        typedef WaterCurrentGaussianProcessPrior This;
        typedef gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector, gtsam::Pose3, gtsam::Vector> Base;

        size_t dof_;
        double delta_t_;

        const WaterCurrentGrid& wcg_;

    public:

        WaterCurrentGaussianProcessPrior() : wcg_(WaterCurrentGrid()){}	/* Default constructor only for serialization */

        /// Constructor
        /// @param delta_t is the time between the two states
        WaterCurrentGaussianProcessPrior(gtsam::Key poseKey1, gtsam::Key velKey1,
                                gtsam::Key poseKey2, gtsam::Key velKey2,
                                const WaterCurrentGrid& wcg,
                                double delta_t, const gtsam::SharedNoiseModel& Qc_model) :
                Base(gtsam::noiseModel::Gaussian::Covariance(calcQ(getQc(Qc_model), delta_t)),
                     poseKey1, velKey1, poseKey2, velKey2),
                            dof_(Qc_model->dim()), delta_t_(delta_t), wcg_(wcg){}

        virtual ~WaterCurrentGaussianProcessPrior() {}


        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

        /// factor error function
        gtsam::Vector evaluateError(
                const gtsam::Pose3& pose1, const gtsam::Vector& vel1,
                const gtsam::Pose3& pose2, const gtsam::Vector& vel2,
                boost::optional<gtsam::Matrix&> H1 = boost::none,
                boost::optional<gtsam::Matrix&> H2 = boost::none,
                boost::optional<gtsam::Matrix&> H3 = boost::none,
                boost::optional<gtsam::Matrix&> H4 = boost::none) const {

            using namespace gtsam;

            Matrix Hinv, Hcomp1, Hcomp2, Hlogmap;
            Vector r;
            if (H1 || H2 || H3 || H4)
                r = traits<gtsam::Pose3>::Logmap(traits<gtsam::Pose3>::Compose(traits<gtsam::Pose3>::Inverse(pose1, Hinv),
                                                         pose2, Hcomp1, Hcomp2), Hlogmap);
            else

                r = traits<gtsam::Pose3>::Logmap(traits<gtsam::Pose3>::Compose(traits<gtsam::Pose3>::Inverse(pose1, Hinv), pose2));

            // jacobians
            if (H1) *H1 = (Matrix(2*dof_, dof_) << Hlogmap * Hcomp1 * Hinv, Matrix::Zero(dof_, dof_)).finished();
            if (H2) *H2 = (Matrix(2*dof_, dof_) << -delta_t_ * Matrix::Identity(dof_, dof_), -Matrix::Identity(dof_, dof_)).finished();
            if (H3) *H3 = (Matrix(2*dof_, dof_) << Hlogmap * Hcomp2, Matrix::Zero(dof_, dof_)).finished();
            if (H4) *H4 = (Matrix(2*dof_, dof_) << Matrix::Zero(dof_, dof_), Matrix::Identity(dof_, dof_)).finished();

            gtsam::Vector6 real_v = wcg_.getVehicleVelocityCurrentLocalFrame(pose1, vel1);

            return (Vector(2*dof_) << (r - real_v * delta_t_), (vel2 - vel1)).finished();
        }

        /** number of variables attached to this factor */
        size_t size() const {
            return 4;
        }

        /** equals specialized to this factor */
        virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
            const This *e =  dynamic_cast<const This*> (&expected);
            return e != NULL && Base::equals(*e, tol) && fabs(this->delta_t_ - e->delta_t_) < tol;
        }

        /** print contents */
        void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
            std::cout << s << "4-way Gaussian Process Facto on Lie<" << dof_ << ">" << std::endl;
            Base::print("", keyFormatter);
        }

    private:

        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int version) {
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
            ar & BOOST_SERIALIZATION_NVP(dof_);
            ar & BOOST_SERIALIZATION_NVP(delta_t_);
        }

    }; // GaussianProcessPriorLie

} // namespace gpmp2


