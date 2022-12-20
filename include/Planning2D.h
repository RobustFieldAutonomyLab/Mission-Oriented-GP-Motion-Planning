//
// Created by rfal on 11/18/22.
//

#ifndef GPMP_STR_PLANNING2D_H
#define GPMP_STR_PLANNING2D_H
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>

#include "../gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h"
#include "../gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h"
#include "../gpmp2/dynamics/VehicleDynamicsFactorPose2.h"
#include "../gpmp2/gp/GaussianProcessPriorPose2.h"
#include "../gpmp2/kinematics/Pose2MobileBase.h"
#include "../gpmp2/kinematics/mobileBaseUtils.h"

#include "Planning.h"
#include "SignedDistanceField.h"

using namespace gtsam;
using namespace gpmp2;


class Planning2D : public Planning<Pose2MobileBaseModel, PlanarSDF>{
public:
    explicit Planning2D(bool use_vehicle_dynamics = true,
               double epsilon_dist = 2,
               double dynamics_sigma = 0.1,
               double cost_sigma = 0.01,
               double vehicle_size = 0.2,
               int check_inter = 5);
    void buildMap(double cell_size, const Point2& origin, Matrix planarSDF);
    std::vector<Pose2> optimize(vector<Pose2> poses,
                                vector<Vector> vels,
                                double delta_t);

private:
    typedef Planning<Pose2MobileBaseModel, PlanarSDF> Base;

    bool _use_vehicle_dynamics;
    double _dynamics_sigma;

};


#endif //GPMP_STR_PLANNING2D_H
