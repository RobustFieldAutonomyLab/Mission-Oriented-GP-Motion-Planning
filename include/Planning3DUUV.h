
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

#include "../gpmp2/mission/SeafloorFactorPose3MobileBase.h"
#include "../gpmp2/mission/SeafloorFactorGPPose3MobileBase.h"
#include "../gpmp2/gp/WaterCurrentGaussianProcessPrior.h"
#include "../gpmp2/mission/SealevelFactorPose3MobileBase.h"
#include "../gpmp2/mission/SealevelFactorGPPose3MobileBase.h"

#include "Planning.h"
#include "SignedDistanceField.h"
#include <gtsam/nonlinear/ISAM2.h>


using namespace gtsam;
using namespace gpmp2;

struct Planning3DUUVParameter{

    //dynamics
    bool use_vehicle_dynamics = true;
    double dynamics_sigma = 0.1;

    //obstacle
    double obstacle_epsilon_dist = 0.2;
    double obstacle_cost_sigma = 0.1;

    //RobotModel
    double vehicle_size = 0.2;
    vector<double> Qc = {0.01, 0.01, 0.001, 0.001, 0.001, 0.001};
    int check_inter = 5;

    //Seafloor
    bool seafloor_mission = true;
    double seafloor_dist = 1;
    double seafloor_cost_sigma = 2;

    //Sealevel
    bool sealevel_mission = false;
    double sealevel_dist = 0;
    double sealevel_cost_sigma = 0;

    //Water current
    bool use_current = false;

    int max_iter = 1000;
};


class Planning3DUUV : public Planning<Pose3MobileBaseModel, SignedDistanceField>{
public:
    explicit Planning3DUUV(Planning3DUUVParameter param);
    Matrix buildMap(double cell_size, double cell_size_z,
                    Point3 origin, Matrix seafloor_map,
                    double sea_level = 0, bool use_boundary = false);
    Matrix buildMap(double cell_size, double cell_size_z,
                    Point3 origin, Matrix seafloor_map,
                    double sea_level, string sdf_path);
    std::vector<Pose3> optimize(vector<Pose3> poses,
                                 vector<Vector> vels,
                                 double delta_t,
                                 string file_path);
    void buildCurrentGrid(double cell_size, double cell_size_z, Point3 origin,
                          vector<Matrix> grid_u, vector<Matrix> grid_v);

    void savePath(string file, Values v, double error, double time);

    void updateSeafloorMissionStatus(bool seafloor_mission);
    void updateSealevelMissionStatus(bool sealevel_mission);

private:
    typedef Planning<Pose3MobileBaseModel, SignedDistanceField> Base;

    Seafloor *sf;
    bool _seafloor_mission;
    double _seafloor_dist;
    double _seafloor_cost_sigma;

    //Sealevel
    bool _sealevel_mission;
    double _sealevel_dist;
    double _sealevel_cost_sigma;
    double _sea_level;

    bool _use_vehicle_dynamics;
    double _dynamics_sigma;//the smaller, the stronger the constraint

    WaterCurrentGrid *wcg;
    bool _use_current;

    int _max_iter;
    int _total_time_step;

};

#endif //GPMP_STR_PLANNING3DUUV_H