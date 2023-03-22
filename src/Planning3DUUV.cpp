#include "Planning3DUUV.h"

Planning3DUUV::Planning3DUUV(Planning3DUUVParameter param):
        _use_vehicle_dynamics(param.use_vehicle_dynamics), _dynamics_sigma(param.dynamics_sigma),
        _seafloor_mission(param.seafloor_mission), _seafloor_dist(param.seafloor_dist),
        _seafloor_cost_sigma(param.seafloor_cost_sigma), _use_current(param.use_current),
        _max_iter(param.max_iter),
        Base(6, param.check_inter, param.Qc,
             param.obstacle_cost_sigma,
             param.obstacle_epsilon_dist){

    double spheres_data[] = {0.0, 0.0, 0.0, 0.0, param.vehicle_size};
    BodySphereVector sphere_vec;

    sphere_vec.emplace_back(BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1], spheres_data[2], spheres_data[3])));

    Pose3MobileBase abs_robot;

    robot = new Pose3MobileBaseModel(abs_robot, sphere_vec);
}
Matrix Planning3DUUV::buildMap(double cell_size, double cell_size_z,
              Point3 origin, Matrix seafloor_map,
              double sea_level, bool use_boundary){
    sdf = buildSDF(cell_size, cell_size_z,
                   origin, seafloor_map,
                   sea_level, use_boundary);
    sf = new Seafloor(origin, cell_size, seafloor_map);
    return seafloor_map;
}

Matrix Planning3DUUV::buildMap(double cell_size, double cell_size_z,
                               Point3 origin, Matrix seafloor_map,
                               double sea_level, string sdf_path){
    sdf = loadSDF(cell_size, cell_size_z, origin, seafloor_map, sea_level, sdf_path);
    sf = new Seafloor(origin, cell_size, seafloor_map);
    return seafloor_map;
}


void Planning3DUUV::buildCurrentGrid(double cell_size, double cell_size_z, Point3 origin,
                      vector<Matrix> grid_u, vector<Matrix> grid_v){
    int rows = grid_u[0].rows();
    int cols = grid_u[0].cols();
    int z = grid_u.size();

    wcg = new WaterCurrentGrid(origin, cell_size, cell_size_z, rows, cols, z,
                                       grid_u, grid_v);
}

std::vector<Pose3> Planning3DUUV::optimize(vector<Pose3> poses,
                            vector<Vector> vels,
                            double delta_t,
                            string file_path){
    NonlinearFactorGraph graph;

    _total_time_step = poses.size() - 1;
    double total_time_sec = delta_t * _total_time_step;
    int total_check_step = (_check_inter + 1)* _total_time_step;

    Key key_pos, key_vel, key_pos1, key_pos2, key_vel1, key_vel2;

    double tau;

    Values init_values;

    for(int i = 0; i <= _total_time_step; i++){
        key_pos = Symbol('x', i);
        key_vel = Symbol('v', i);

        init_values.insert(key_pos, poses[i]);
        init_values.insert(key_vel, vels[i]);
    }

    for(int i = 0; i <= _total_time_step; i++){
        key_pos = Symbol('x', i);
        key_vel = Symbol('v', i);
        if(i == 0) {
            graph.add(PriorFactor<Pose3>(key_pos, poses[0], pose_fix));
            graph.add(PriorFactor<Vector>(key_vel, vels[0], vel_fix));
        }
        else if(i == _total_time_step){
            graph.add(PriorFactor<Pose3>(key_pos, poses[_total_time_step], pose_fix));
            graph.add(PriorFactor<Vector>(key_vel, vels[_total_time_step], vel_fix));
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

            if(_use_current){
                graph.add(WaterCurrentGaussianProcessPrior(
                        key_pos1, key_vel1,
                        key_pos2, key_vel2, *wcg, delta_t, Qc_model));
            }
            else{
                graph.add(GaussianProcessPriorPose3(
                        key_pos1, key_vel1,
                        key_pos2, key_vel2, delta_t, Qc_model));
            }


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
                if(_use_current){
                    graph.add(ObstacleSDFFactorGPPose3MobileBase (
                            key_pos1, key_vel1, key_pos2, key_vel2,
                            *robot, *sdf, *wcg, _cost_sigma,
                            _epsilon_dist, Qc_model, delta_t, tau));
                }
                else{
                    graph.add(ObstacleSDFFactorGPPose3MobileBase (
                            key_pos1, key_vel1, key_pos2, key_vel2,
                            *robot, *sdf, _cost_sigma,
                            _epsilon_dist, Qc_model, delta_t, tau));
                }


                if(_seafloor_mission){
                    graph.add(SeafloorFactorGPPose3MobileBase (
                            key_pos1, key_vel1, key_pos2, key_vel2,
                            *robot, *sf, *wcg,_seafloor_cost_sigma,
                            _seafloor_dist, Qc_model, delta_t, tau));
                }
                else{
                    graph.add(SeafloorFactorGPPose3MobileBase (
                            key_pos1, key_vel1, key_pos2, key_vel2,
                            *robot, *sf, _seafloor_cost_sigma,
                            _seafloor_dist, Qc_model, delta_t, tau));
                }

            }
        }
    }

    DoglegOptimizer optimizer(graph, init_values);
    DoglegParams params;
    params.maxIterations = _max_iter;
    Values result = optimizer.optimize();
    vector <Pose3> p;
    savePath(file_path, result, graph.error(result)/_total_time_step);
    for (int i = 0; i <= _total_time_step; i++ ){
        Pose3 pose = result.at<Pose3>(Symbol('x', i));
        p.push_back(pose);
    }
    return p;
}

void Planning3DUUV::savePath(string filename, Values v, double error){
    std::fstream file;
    file.open(filename, std::ios::out);

    v.print("Final Result:\n");
    file <<"total error: "<< error <<endl;
    file <<"x, y, z, roll, pitch, yaw, dx, dy, dz, droll, dpitch, dyaw"<<endl;
    for (int i = 0; i <= _total_time_step; i++ ){
        Pose3 pose = v.at<Pose3>(Symbol('x', i));
        Vector vel = v.at<Vector>(Symbol('v', i));
        if(_use_current){
            cout<< "Velocity " << i <<": "<< vel<<endl;
        }
        file << pose.x()<<", "<<pose.y()<<", "<<pose.z()<<", "
                << pose.rotation().roll()<<", "<<pose.rotation().pitch()<<", "<<pose.rotation().yaw()<<", "
                << vel(3)<<", "<<vel(4)<<", "<<vel(5)<<", "
                << vel(0)<<", "<<vel(1)<<", "<<vel(2)<< endl;
    }
    file.close();
}