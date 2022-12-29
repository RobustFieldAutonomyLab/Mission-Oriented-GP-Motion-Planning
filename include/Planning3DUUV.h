
#ifndef GPMP_STR_PLANNING3DUUV_H
#define GPMP_STR_PLANNING3DUUV_H
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>

#include "../gpmp2/obstacle/ObstacleSDFFactorPose3MobileBase.h"
#include "../gpmp2/obstacle/ObstacleSDFFactorGPPose3MobileBase.h"
#include "../gpmp2/dynamics/VehicleDynamicsFactorPose3.h"
#include "../gpmp2/gp/GaussianProcessPriorPose3.h"
#include "../gpmp2/kinematics/Pose3MobileBaseModel.h"
#include "../gpmp2/kinematics/mobileBaseUtils.h"

#include "Planning.h"
#include "SignedDistanceField.h"

using namespace gtsam;
using namespace gpmp2;


class Planning3DUUV : public Planning<Pose3MobileBaseModel, SignedDistanceField>{
public:
    explicit Planning3DUUV(bool use_vehicle_dynamics = true,
               double epsilon_dist = 5,
               double dynamics_sigma = 0.1,
               double cost_sigma = 2,
               double vehicle_size = 0.2,
               int check_inter = 5);
    void buildMap(double cell_size, double cell_size_z, Point3 origin, Matrix seafloor_map);
    std::vector<Pose3> optimize(vector<Pose3> poses,
                                 vector<Vector> vels,
                                 double delta_t);

private:
    typedef Planning<Pose3MobileBaseModel, SignedDistanceField> Base;

    bool _use_vehicle_dynamics;
    double _dynamics_sigma;//the smaller, the stronger the constraint

};

#endif //GPMP_STR_PLANNING3DUUV_H