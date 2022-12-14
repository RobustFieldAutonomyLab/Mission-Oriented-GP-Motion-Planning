//
// Created by rfal on 11/22/22.
//

#ifndef GPMP_STR_PLANNING3D_H
#define GPMP_STR_PLANNING3D_H
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "../gpmp2/kinematics/PointRobot.h"
#include "../gpmp2/kinematics/PointRobotModel.h"
#include "../gpmp2/gp/GaussianProcessPriorLinear.h"
#include "../gpmp2/gp/GaussianProcessInterpolatorLinear.h"
#include "../gpmp2/obstacle/SignedDistanceField.h"
#include "../gpmp2/obstacle/ObstacleSDFFactor.h"
#include "../gpmp2/obstacle/ObstacleSDFFactorGP.h"

#include "SignedDistanceField.h"


using namespace gtsam;
using namespace gpmp2;


class Planning3D {
public:
    explicit Planning3D(double epsilon_dist = 2,
               double cost_sigma = 2,
               double vehicle_size = 2,
               int check_inter = 5);
    void buildMap(double cell_size, double cell_size_z, Point3 origin, Matrix seafloor_map);
    std::vector<Vector> optimize(std::vector<Vector> poses,
                                 std::vector<Vector> vels,
                                 double delta_t);
    std::vector<std::tuple<double, double, double>>
        pyoptimize(vector<std::tuple<double, double, double>> poses,
               vector<std::tuple<double, double, double>> vels,
               double delta_t);

private:
    int _check_inter;

    double _cost_sigma;
    double _epsilon_dist;

    SignedDistanceField * sdf;
    PointRobotModel *robot;

    Matrix3 Qc = 1 * Matrix::Identity(3, 3);

    noiseModel::Gaussian::shared_ptr Qc_model;

    noiseModel::Isotropic::shared_ptr pose_fix;

    noiseModel::Isotropic::shared_ptr vel_fix;

};


#endif //GPMP_STR_PLANNING3D_H
