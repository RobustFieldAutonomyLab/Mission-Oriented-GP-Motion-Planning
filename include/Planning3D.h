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

#include "Planning.h"
#include "SignedDistanceField.h"


using namespace gtsam;
using namespace gpmp2;


class Planning3D : public Planning<PointRobotModel, SignedDistanceField>{
public:
    explicit Planning3D(double epsilon_dist = 2,
               double cost_sigma = 2,
               double vehicle_size = 2,
               int check_inter = 5);
    void buildMap(Matrix seafloor_map, Point3 origin, double cell_size, double cell_size_z);
    std::vector<Vector> optimize(std::vector<Vector> poses,
                                 std::vector<Vector> vels,
                                 double delta_t);
private:
    typedef Planning<PointRobotModel, SignedDistanceField> Base;

};


#endif //GPMP_STR_PLANNING3D_H
