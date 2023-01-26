#include "OMPLHelper.h"


int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    OMPLParam params;
    params.method_ = RRTStar;
    params.cell_size_ = 1;
    params.cell_size_z_ = 1;
    params.origin_ = gtsam::Point3(0, 0, -4243);
    params.dist_sdf_ = 3;
    params.dist_sf_ = 5;
    params.w_vd_ = 1;
    params.w_sf_ = 1;
    params.w_sdf_ = 1;
    params.cost_thres_ = 100;

    OMPLHelper env("../data/depth_grid2.csv", params);

    if (env.plan(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(5, 5, -4220)),
                 gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(45, 45, -4182))) )
    {
        env.recordSolution();
    }

    return 0;
}