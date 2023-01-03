#include "Planning3DUUV.h"

Planning3DUUV::Planning3DUUV(Planning3DUUVParameter param):
        _use_vehicle_dynamics(param.use_vehicle_dynamics), _dynamics_sigma(param.dynamics_sigma),
        _seafloor_mission(param.seafloor_mission), _seafloor_dist(param.seafloor_dist),
        _seafloor_cost_sigma(param.seafloor_cost_sigma),
        Base(6, param.check_inter,
             param.obstacle_cost_sigma,
             param.obstacle_epsilon_dist){

    double spheres_data[] = {0.0, 0.0, 0.0, 0.0, param.vehicle_size};
    BodySphereVector sphere_vec;

    sphere_vec.emplace_back(BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1], spheres_data[2], spheres_data[3])));

    Pose3MobileBase abs_robot;

    robot = new Pose3MobileBaseModel(abs_robot, sphere_vec);

}
void Planning3DUUV::buildMap(double cell_size, double cell_size_z,
              Point3 origin, Matrix seafloor_map){
    sdf = buildSDF(cell_size, cell_size_z, origin, seafloor_map);
    sf = new Seafloor(origin, cell_size, seafloor_map);
}
std::vector<Pose3> Planning3DUUV::optimize(vector<Pose3> poses,
                            vector<Vector> vels,
                            double delta_t){
    NonlinearFactorGraph graph;

    int total_time_step = poses.size() - 1;
    double total_time_sec = delta_t * total_time_step;
    int total_check_step = (_check_inter + 1)* total_time_step;

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
            graph.add(PriorFactor<Pose3>(key_pos, poses[0], pose_fix));
            graph.add(PriorFactor<Vector>(key_vel, vels[0], vel_fix));
        }
        else if(i == total_time_step){
            graph.add(PriorFactor<Pose3>(key_pos, poses[total_time_step], pose_fix));
            graph.add(PriorFactor<Vector>(key_vel, vels[total_time_step], vel_fix));
        }

        if(_use_vehicle_dynamics){
            graph.add(VehicleDynamicsFactorPose3(
                    key_pos, key_vel, _dynamics_sigma));
        }

        if(i > 0){
            key_pos1 = Symbol('x', i-1);
            key_pos2 = Symbol('x', i);
            key_vel1 = Symbol('v', i-1);
            key_vel2 = Symbol('v', i);
            graph.add(GaussianProcessPriorPose3(
                    key_pos1, key_vel1,
                    key_pos2, key_vel2, delta_t, Qc_model));

            graph.add(ObstacleSDFFactorPose3MobileBase (
                    key_pos, *robot, *sdf,
                    _cost_sigma, _epsilon_dist));

            if(_seafloor_mission){
                graph.add(SeafloorFactorPose3MobileBase(
                        key_pos, *robot, *sf,
                        _seafloor_cost_sigma,
                        _seafloor_dist));
            }

            for(int j = 1; j <= _check_inter+1; j++){
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstacleSDFFactorGPPose3MobileBase (
                        key_pos1, key_vel1, key_pos2, key_vel2,
                        *robot, *sdf, _cost_sigma,
                        _epsilon_dist, Qc_model, delta_t, tau));
            }
        }
    }

//    graph.print("\nFactor Graph:\n");
//    init_values.print("\nInitial Values:\n");

//    GaussNewtonParams parameters;
    //parameters.relativeErrorTol = 1e-5;
    //parameters.maxIterations = 100;
//    graph.print();
    DoglegOptimizer optimizer(graph, init_values);
    Values result = optimizer.optimize();
    result.print("Final Result:\n");

    std::vector<Pose3> out;
    for (int i = 0; i <= total_time_step; i++ ){
        Pose3 vel = result.at<Pose3>(Symbol('x', i));
        out.push_back(vel);
    }
    return out;

}