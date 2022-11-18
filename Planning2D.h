//
// Created by rfal on 11/18/22.
//

#ifndef GPMP_STR_PLANNING2D_H
#define GPMP_STR_PLANNING2D_H
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>

#include "obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h"
#include "obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h"
#include "dynamics/VehicleDynamicsFactorPose2.h"
#include "gp/GaussianProcessPriorPose2.h"
#include "kinematics/Pose2MobileBase.h"
#include "kinematics/mobileBaseUtils.h"


using namespace gtsam;
using namespace gpmp2;


class Planning2D {
public:
    Planning2D(bool use_vehicle_dynamics = true,
               double epsilon_dist = 0.2,
               double dynamics_sigma = 0.001,
               double cost_sigma = 0.01,
               double vehicle_size = 0.2,
               int check_inter = 5);
    void buildMap(double cell_size, const Point2& origin, int map_size_x, int map_size_y);
    std::vector<Pose2> optimize(vector<Pose2> poses,
                                vector<Vector> vels,
                                double delta_t);


private:
    bool _use_vehicle_dynamics;

    int _check_inter;

    double _cost_sigma;
    double _epsilon_dist;
    double _dynamics_sigma;

    PlanarSDF *sdf;
    Pose2MobileBaseModel *robot;

    Matrix3 Qc = 1 * Matrix::Identity(3, 3);

    noiseModel::Gaussian::shared_ptr Qc_model;

    noiseModel::Isotropic::shared_ptr pose_fix;

    noiseModel::Isotropic::shared_ptr vel_fix;

};


#endif //GPMP_STR_PLANNING2D_H
