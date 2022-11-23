//
// Created by rfal on 11/22/22.
//

#include "Planning3D.h"

Planning3D::Planning3D(double epsilon_dist,
                       double cost_sigma,
                       double vehicle_size, int check_int) {
    double spheres_data[] = {0.0, 0.0, 0.0, 0.0, vehicle_size};
    BodySphereVector sphere_vec;

    sphere_vec.push_back(BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1], spheres_data[2], spheres_data[3])));

    robot = new PointRobotModel(PointRobot(), sphere_vec);

    Qc = 1 * Matrix::Identity(3, 3);

    Qc_model = noiseModel::Gaussian::Covariance(Qc);

    pose_fix = noiseModel::Isotropic::Sigma(3, 0.0001);

    vel_fix = noiseModel::Isotropic::Sigma(3, 0.0001);

}

void Planning3D::buildMap(double cell_size, Point3 origin, int map_size_x, int map_size_y, int map_size_z) {
    sdf = new SignedDistanceField(origin, cell_size, map_size_x, map_size_y, map_size_z);
    for (int z=0; z < map_size_z; z++){
        Matrix data;
        data = Matrix (map_size_x, map_size_y);
        sdf->initFieldData(z, data);
    }
}

std::vector<Vector> Planning3D::optimize(std::vector<Vector> poses,
                             std::vector<Vector> vels,
                             double delta_t){
    NonlinearFactorGraph graph;

    int total_time_step = poses.size() - 1;
    double total_time_sec = delta_t * total_time_step;
    int total_check_inter = (_check_inter + 1)* total_time_step;

    Key key_pos, key_vel, key_pos1, key_pos2, key_vel1, key_vel2;

    double tau;

    Values init_values;

    for(int i = 0; i <= total_time_step; i++){
        key_pos = Symbol('x', i);
        key_vel = Symbol('v', i);

        init_values.insert(key_pos, poses[i]);
        init_values.insert(key_vel, vels[i]);
    }

    for(int i = 0; i <= total_time_step; i++){
        key_pos = Symbol('x', i);
        key_vel = Symbol('v', i);
        if(i == 0) {
            graph.add(PriorFactor<Vector>(key_pos, poses[0], pose_fix));
            graph.add(PriorFactor<Vector>(key_vel, vels[0], vel_fix));
        }
        else if(i == total_time_step){
            graph.add(PriorFactor<Vector>(key_pos, poses[total_time_step], pose_fix));
            graph.add(PriorFactor<Vector>(key_vel, vels[total_time_step], vel_fix));
        }

        graph.add(ObstacleSDFFactor<PointRobotModel> (
                key_pos, *robot, *sdf, _cost_sigma, _epsilon_dist));

        if(i > 0){
            key_pos1 = Symbol('x', i-1);
            key_pos2 = Symbol('x', i);
            key_vel1 = Symbol('v', i-1);
            key_vel2 = Symbol('v', i);
            graph.add(GaussianProcessPriorLinear(
                    key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));

            for(int j = 1; j <= _check_inter; j++){
                tau = j * (total_time_sec / total_check_inter);
                graph.add(ObstacleSDFFactorGP<PointRobotModel, GaussianProcessInterpolatorLinear>(
                        key_pos1, key_vel1, key_pos2, key_vel2, *robot, *sdf, _cost_sigma, _epsilon_dist, Qc_model, delta_t, tau));
            }
        }
    }

//    graph.print("\nFactor Graph:\n");
//    init_values.print("\nInitial Values:\n");

    GaussNewtonParams parameters;
    //parameters.relativeErrorTol = 1e-5;
    //parameters.maxIterations = 100;
    GaussNewtonOptimizer optimizer(graph, init_values, parameters);
    Values result = optimizer.optimize();
//    result.print("Final Result:\n");

    std::vector<Vector> out;
    for (int i = 0; i <= total_time_step; i++ ){
        Vector vel = result.at<Vector>(Symbol('v', i));
        out.push_back(vel);
    }
    return out;
}