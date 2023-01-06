//
// Created by rfal on 12/5/22.
//

#include "Pose3MobileBase.h"
#include "mobileBaseUtils.h"

#include <iostream>

using namespace gtsam;
using namespace std;

namespace gpmp2{
/* ************************************************************************** */
    void Pose3MobileBase::forwardKinematics(
            const gtsam::Pose3& p, boost::optional<const gtsam::Vector&> v,
                           std::vector<gtsam::Pose3>& px, boost::optional<std::vector<gtsam::Vector3>&> vx,
                           boost::optional<std::vector<gtsam::Matrix>&> J_px_p,
                           boost::optional<std::vector<gtsam::Matrix>&> J_vx_p,
                           boost::optional<std::vector<gtsam::Matrix>&> J_vx_v) const{
        if (v)
            throw runtime_error("[Pose3MobileBase] TODO: velocity not implemented");

        if (!v && (vx || J_vx_p || J_vx_v))
            throw runtime_error("[Pose3MobileBase] ERROR: only ask for velocity in workspace given velocity in "
                                "configuration space");

        // allocate space
        px.resize(nr_links());
        if (vx) vx->resize(nr_links());
        if (J_px_p) J_px_p->assign(nr_links(), Matrix::Zero(6, dof()));
        if (J_vx_p) J_vx_p->assign(nr_links(), Matrix::Zero(3, dof()));
        if (J_vx_v) J_vx_v->assign(nr_links(), Matrix::Zero(3, dof()));

        // assign values
        px[0] = p;
        if (J_px_p || J_vx_p || J_vx_v) {
            if (J_px_p) {
//                const gtsam::Matrix3 Hzrot3 = gtsam::Rot3::ExpmapDerivative(
//                        gtsam::Vector3(p.rotation().roll(), p.rotation().pitch(), p.rotation().yaw()));
//                (*J_px_p)[0].setZero();
//                (*J_px_p)[0].block<3, 3>(0, 0) = Hzrot3;
//                (*J_px_p)[0].block<3, 3>(3, 3) = gtsam::Matrix3::Identity();
//                Vector6 v6;
//                v6 << Vector3(p.rotation().roll(), p.rotation().pitch(), p.rotation().yaw()),
//                                Vector3(p.translation().x(), p.translation().y(), p.translation().z());
//                const gtsam::Matrix6 Hveh_base = gtsam::Pose3::ExpmapDerivative(v6);
//                (*J_px_p)[0].block<6, 6>(0, 0) = Hveh_base.transpose();
//                cout<<"J_px_p: "<<(*J_px_p)[0]<<endl;
                //This is my final decision.
                (*J_px_p)[0].block<6, 6>(0, 0)  = I_6x6;
//                cout<<"ExpmapDerivative: "<<Hveh_base<<endl;
            }
            if (vx) {
                Vector6 vx_tmp;
                //TODO: decide the right v
                (*vx)[0] = Vector3((*v)[3], (*v)[4], (*v)[5]);
                // (*J_vx_p)[0] is zero
                if (J_vx_v)
                    (*J_vx_v)[0].block<3, 3>(3, 0) = Matrix3::Identity();
            }
        }

    }
}