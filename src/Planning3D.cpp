//
// Created by rfal on 11/22/22.
//

#include "Planning3D.h"

Planning3D::Planning3D(double epsilon_dist,
                       double cost_sigma,
                       double vehicle_size, int check_inter):
                       _cost_sigma(cost_sigma),
                       _epsilon_dist(epsilon_dist),
                       _check_inter(check_inter){
    double spheres_data[] = {0.0, 0.0, 0.0, 0.0, vehicle_size};
    BodySphereVector sphere_vec;

    sphere_vec.push_back(BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1], spheres_data[2], spheres_data[3])));

    robot = new PointRobotModel(PointRobot(3,1), sphere_vec);

    Qc = 1 * Matrix::Identity(3, 3);

    Qc_model = noiseModel::Gaussian::Covariance(Qc);

    pose_fix = noiseModel::Isotropic::Sigma(3, 0.0001);

    vel_fix = noiseModel::Isotropic::Sigma(3, 0.0001);

}

void Planning3D::buildMap(double cell_size, double cell_size_z, Point3 origin, Matrix seafloor_map) {
    auto s_max = seafloor_map.maxCoeff();
    auto s_min = seafloor_map.minCoeff();
    int rows = seafloor_map.rows();
    int cols = seafloor_map.cols();

    int z_level = int( (s_max - s_min) / cell_size_z ) + 3;

    sdf = new SignedDistanceField(origin, cell_size, cell_size_z,
                                  rows, cols, z_level);
    vector<Matrix> data_3D;

    for (int z=0; z < z_level; z++){
        Matrix data;
        data.setZero(seafloor_map.rows(), seafloor_map.cols());
        for (int i=0;i<rows;i++){
            for(int j=0;j<cols;j++){
                if ( z * cell_size_z + origin.z() < seafloor_map(i,j) ){

                    data(i,j) = 1;
                }
//                cout<< z * cell_size_z + origin.z() <<endl;
            }
        }
        data_3D.push_back(data);
    }
    vector<Matrix> fields = signedDistanceField3D(data_3D, cell_size);
    int level = 0;
    for (auto field : fields){
//        plotEvidenceMap2D(field, 0, 0, 1);
//        sleep(1);
        sdf->initFieldData(level, field);
        level ++;
//        print(field);
    }

}

std::vector<Vector> Planning3D::optimize(std::vector<Vector> poses,
                             std::vector<Vector> vels,
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
            graph.add(PriorFactor<Vector>(key_pos, poses[0], pose_fix));
            graph.add(PriorFactor<Vector>(key_vel, vels[0], vel_fix));
        }
        else if(i == total_time_step){
            graph.add(PriorFactor<Vector>(key_pos, poses[total_time_step], pose_fix));
            graph.add(PriorFactor<Vector>(key_vel, vels[total_time_step], vel_fix));
        }

        if(i > 0){
            key_pos1 = Symbol('x', i-1);
            key_pos2 = Symbol('x', i);
            key_vel1 = Symbol('v', i-1);
            key_vel2 = Symbol('v', i);
            graph.add(GaussianProcessPriorLinear(
                    key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));

            graph.add(ObstacleSDFFactor<PointRobotModel> (
                    key_pos, *robot, *sdf, _cost_sigma, _epsilon_dist));

            for(int j = 1; j <= _check_inter+1; j++){
                tau = j * (total_time_sec / total_check_step);
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
    DoglegOptimizer optimizer(graph, init_values);
    Values result = optimizer.optimize();
//    result.print("Final Result:\n");

    std::vector<Vector> out;
    for (int i = 0; i <= total_time_step; i++ ){
        Vector vel = result.at<Vector>(Symbol('x', i));
        out.push_back(vel);
    }
    return out;
}

std::vector<std::tuple<double, double, double>> Planning3D::pyoptimize(
        vector<std::tuple<double, double, double>> poses,
            vector<std::tuple<double, double, double>> vels,
                double delta_t){
    vector<Vector> poses_v;
    vector<Vector> vels_v;
    for (auto pose : poses){
        poses_v.emplace_back(Vector3(std::get<0>(pose),
                                   std::get<1>(pose),
                                   std::get<2>(pose)));
    }
    for (auto vel : vels){
        vels_v.emplace_back(Vector3(std::get<0>(vel),
                                    std::get<1>(vel),
                                    std::get<2>(vel)));
    }
    auto results = optimize(poses_v, vels_v, delta_t);

    vector<std::tuple<double, double, double>> t_results;
    for(auto p : results){
        t_results.emplace_back(make_tuple(p[0], p[1], p[2]));
    }
    return t_results;

}
