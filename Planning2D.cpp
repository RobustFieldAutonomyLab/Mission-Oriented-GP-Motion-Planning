//
// Created by rfal on 11/18/22.
//

#include "Planning2D.h"

Planning2D::Planning2D(bool use_vehicle_dynamics, double epsilon_dist,
           double dynamics_sigma, double cost_sigma,
           double vehicle_size, int check_inter):
        _use_vehicle_dynamics(use_vehicle_dynamics),
        _dynamics_sigma(dynamics_sigma), _cost_sigma(cost_sigma),
        _epsilon_dist(epsilon_dist),
        _check_inter(check_inter){
    double spheres_data[] = {0.0, 0.0, 0.0, 0.0, vehicle_size};
    BodySphereVector sphere_vec;

    sphere_vec.push_back(BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1], spheres_data[2], spheres_data[3])));

    Pose2MobileBase abs_robot;

    robot = new Pose2MobileBaseModel(abs_robot, sphere_vec);

    Qc = 1 * Matrix::Identity(3, 3);

    Qc_model = noiseModel::Gaussian::Covariance(Qc);

    pose_fix = noiseModel::Isotropic::Sigma(3, 0.0001);

    vel_fix = noiseModel::Isotropic::Sigma(3, 0.0001);
}
void Planning2D::buildMap(double cell_size, const Point2& origin, int map_size_x, int map_size_y){
    Matrix data;
    data = Matrix (map_size_x, map_size_y);

    //Construct binary map with static obstacles
//      std::vector<Point2> current_position{Point2(10,10), Point2(20,20)};
//      for(int i = 0; i < current_position.size(); i++)
//      {
//          for(int j = current_position[i].x() - 3; j < current_position[i].x() + 3; j++)
//          {
//              for(int k = current_position[i].y() - 3; k < current_position[i].y() + 3; k++)
//              {
//                  data(j, k) = 255;
//              }
//          }
//      }
    sdf = new PlanarSDF(origin, cell_size, data);
}
std::vector<Pose2> Planning2D::optimize(vector<Pose2> poses,
                            vector<Vector> vels,
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
            graph.add(PriorFactor<Pose2>(key_pos, poses[0], pose_fix));
            graph.add(PriorFactor<Vector>(key_vel, vels[0], vel_fix));
        }
        else if(i == total_time_step){
            graph.add(PriorFactor<Pose2>(key_pos, poses[total_time_step], pose_fix));
            graph.add(PriorFactor<Vector>(key_vel, vels[total_time_step], vel_fix));
        }

        graph.add(ObstaclePlanarSDFFactorPose2MobileBase(
                key_pos, *robot, *sdf, _cost_sigma, _epsilon_dist));

        if(_use_vehicle_dynamics == true){
            graph.add(VehicleDynamicsFactorPose2(
                    key_pos, key_vel, _dynamics_sigma));
        }

        if(i > 0){
            key_pos1 = Symbol('x', i-1);
            key_pos2 = Symbol('x', i);
            key_vel1 = Symbol('v', i-1);
            key_vel2 = Symbol('v', i);
            graph.add(GaussianProcessPriorPose2(
                    key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));

            for(int j = 1; j <= _check_inter; j++){
                tau = j * (total_time_sec / total_check_inter);
                graph.add(ObstaclePlanarSDFFactorGPPose2MobileBase(
                        key_pos1, key_vel1, key_pos2, key_vel2, *robot, *sdf, _cost_sigma, _epsilon_dist, Qc_model, delta_t, tau));
            }
        }
    }

    graph.print("\nFactor Graph:\n");
    init_values.print("\nInitial Values:\n");

    GaussNewtonParams parameters;
    //parameters.relativeErrorTol = 1e-5;
    //parameters.maxIterations = 100;
    GaussNewtonOptimizer optimizer(graph, init_values, parameters);
    Values result = optimizer.optimize();
    result.print("Final Result:\n");

    std::vector<Pose2> out;
    for (int i = 0; i <= total_time_step; i++ ){
        Pose2 pos = result.at<Pose2>(Symbol('x', i));
        out.push_back(pos);
    }
    return out;
}