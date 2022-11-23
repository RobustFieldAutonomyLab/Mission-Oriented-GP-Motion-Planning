//
// Created by rfal on 11/22/22.
//

#ifndef GPMP_STR_PLANNING3D_H
#define GPMP_STR_PLANNING3D_H
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "gpmp2/kinematics/PointRobot.h"
#include "gpmp2/kinematics/PointRobotModel.h"
#include "gpmp2/gp/GaussianProcessPriorLinear.h"
#include "gpmp2/gp/GaussianProcessInterpolatorLinear.h"
#include "gpmp2/obstacle/SignedDistanceField.h"
#include "gpmp2/obstacle/ObstacleSDFFactor.h"
#include "gpmp2/obstacle/ObstacleSDFFactorGP.h"

using namespace gtsam;
using namespace gpmp2;


class Planning3D {
public:
    explicit Planning3D(double epsilon_dist = 0.2,
               double cost_sigma = 0.02,
               double vehicle_size = 0.2,
               int check_inter = 5);
    void buildMap(double cell_size, Point3 origin, int map_size_x, int map_size_y, int map_size_z);
    std::vector<Vector> optimize(std::vector<Vector> poses,
                                 std::vector<Vector> vels,
                                 double delta_t);
private:
    int _check_inter;

    double _cost_sigma;
    double _epsilon_dist;
    double _dynamics_sigma;

    SignedDistanceField * sdf;
    PointRobotModel *robot;

    Matrix3 Qc = 1 * Matrix::Identity(3, 3);

    noiseModel::Gaussian::shared_ptr Qc_model;

    noiseModel::Isotropic::shared_ptr pose_fix;

    noiseModel::Isotropic::shared_ptr vel_fix;

};


#endif //GPMP_STR_PLANNING3D_H
