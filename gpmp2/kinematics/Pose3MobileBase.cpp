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
    void Pose3MobileBase::forwardKinematics(const gtsam::Pose3& p, boost::optional<const gtsam::Vector&> v,
                           std::vector<gtsam::Pose3>& px, boost::optional<std::vector<gtsam::Vector6>&> vx,
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
        if (J_vx_p) J_vx_p->assign(nr_links(), Matrix::Zero(6, dof()));
        if (J_vx_v) J_vx_v->assign(nr_links(), Matrix::Zero(6, dof()));

        // assign values
        Vector6 pv;
        pv << Vector3(p.rotation().roll(), p.rotation().pitch(), p.rotation().yaw()),
                    Vector3(p.translation().x(),p.translation().y(),p.translation().z());
        const Matrix6 Hveh_base = Pose3::ExpmapDerivative(pv);
        px[0] = p;
        if (J_px_p || J_vx_p || J_vx_v) {
            if (J_px_p) (*J_px_p)[0].block<6,6>(0,0) = Hveh_base;
            if (vx) {
                Vector6 vx_tmp;
                //TODO: decide the right v
                vx_tmp << Vector3((*v)[0], (*v)[1], (*v)[2]), Vector3((*v)[3], (*v)[4], (*v)[5]);
                (*vx)[0] = vx_tmp;
                // (*J_vx_p)[0] is zero
                if (J_vx_v)
                    (*J_vx_v)[0].block<6,6>(0,0) = Matrix6::Identity();
            }
        }


    }
}