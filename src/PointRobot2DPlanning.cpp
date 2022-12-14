//
// Created by rfal on 12/1/22.
//
#include <iostream>
#include <cmath>
#include <math.h>
#include <string>
#include <vector>

#include "../gpmp2/obstacle/ObstaclePlanarSDFFactor.h"
#include "../gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h"
#include "../gpmp2/gp/GaussianProcessPriorLinear.h"


#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include "Visualization.h"
#include "SignedDistanceField.h"


using namespace std;
using namespace gtsam;
using namespace gpmp2;


int main(){
    Matrix data, data_ocf;
    Point2 origin(-20.0, -10.0);

    data = Matrix(300, 400);

    double cell_size = 0.1;

    //Construct binary map with static obstacles

    vector<vector<Point2>> obstacles = {
            {get_center(12,10,origin.x(), origin.y(), cell_size),
                get_dim(5, 7, cell_size)},
            {get_center(-7,10,origin.x(), origin.y(), cell_size),
             get_dim(10, 7, cell_size)},
            {get_center(0,-5,origin.x(), origin.y(), cell_size),
             get_dim(10, 5, cell_size)} };

    for (auto current_position : obstacles) {
        int half_size_row = int(floor(current_position[1].x()-1)/2);
        int half_size_col = int(floor(current_position[1].y()-1)/2);

        for (int j = current_position[0].x() - half_size_row - 1;
             j < current_position[0].x() + half_size_row; j++) {
            for (int k = current_position[0].y() - half_size_col - 1;
                 k < current_position[0].y() + half_size_col; k++) {
                data(j, k) = 1;
            }
        }
    }
    auto data_field = signedDistanceField2D(data, cell_size);
    PlanarSDF sdf(origin, cell_size, data_field);
// settings
    double total_time_sec = 10.0;
    int total_time_step = 20;
    double total_check_step = 50.0;
    double delta_t = total_time_sec / total_time_step;
    int check_inter = int(total_check_step / total_time_step - 1);

    bool use_GP_inter = true;
    double spheres_data[] = {0.0, 0.0, 0.0, 0.0, 1.5};
    BodySphereVector sphere_vec;

    sphere_vec.push_back(BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1], spheres_data[2], spheres_data[3])));

    PointRobot pR = PointRobot(2,1);
    auto pR_model = PointRobotModel(pR, sphere_vec);

// GP
    auto Qc = 1 * Matrix::Identity(2, 2);
    noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);

// Obstacle avoid settings
    double cost_sigma = 0.5;
    double epsilon_dist = 4.0;

// prior to start/goal
    noiseModel::Isotropic::shared_ptr pose_fix = noiseModel::Isotropic::Sigma(2, 0.0001);

    noiseModel::Isotropic::shared_ptr vel_fix = noiseModel::Isotropic::Sigma(2, 0.0001);

// start and end conf
    Vector start_conf = Vector2(0, 0);
    Vector start_vel = Vector2(0, 0);
    Vector end_conf = Vector2(17, 14);
    Vector end_vel = Vector2(0, 0);
    Vector avg_vel = (end_conf / total_time_step) / delta_t;

// plot param
    double pause_time = total_time_sec / total_time_step;

    NonlinearFactorGraph graph;
    Values init_values;

    for (int i = 0; i < total_time_step + 1; i++) {
        Key key_pos = symbol('x', i);
        Key key_vel = symbol('v', i);

// initialize as straight line in conf space
        Vector pose = start_conf * float(total_time_step - i) / float(
                total_time_step
        ) + end_conf * i / float(total_time_step);
        auto vel = avg_vel;
//        print(pose)
        init_values.insert(key_pos, pose);
        init_values.insert(key_vel, vel);

// start / end priors
        if (i == 0){
            graph.push_back(PriorFactor<Vector>(key_pos, start_conf, pose_fix));
            graph.push_back(PriorFactor<Vector>(key_vel, start_vel, vel_fix));
        }
        else if(i == total_time_step){
            graph.push_back(PriorFactor<Vector>(key_pos, end_conf, pose_fix));
            graph.push_back(PriorFactor<Vector>(key_vel, end_vel, vel_fix));
        }


// GP priors and cost factor
        if (i > 0) {
            auto key_pos1 = symbol('x', i - 1);
            auto key_pos2 = symbol('x', i);
            auto key_vel1 = symbol('v', i - 1);
            auto key_vel2 = symbol('v', i);

            auto temp = GaussianProcessPriorLinear(
                    key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model
            );
            graph.push_back(temp);

// cost factor
            graph.push_back(
                    ObstaclePlanarSDFFactor<PointRobotModel>(
                            key_pos, pR_model, sdf, cost_sigma, epsilon_dist
                    )
            );

// GP cost factor
            if (use_GP_inter and check_inter > 0) {
                for (int j = 1; j < check_inter + 1; j++) {
                    double tau = j * (total_time_sec / total_check_step);
                    graph.add(
                            ObstaclePlanarSDFFactorGPPointRobot(
                                    key_pos1,
                                    key_vel1,
                                    key_pos2,
                                    key_vel2,
                                    pR_model,
                                    sdf,
                                    cost_sigma,
                                    epsilon_dist,
                                    Qc_model,
                                    delta_t,
                                    tau)
                            );
                }
            }

        }
    }
    bool use_trustregion_opt = true;
    if (use_trustregion_opt){
        DoglegParams parameters;
        //parameters.relativeErrorTol = 1e-5;
        //parameters.maxIterations = 100;
        DoglegOptimizer optimizer(graph, init_values);
        Values result = optimizer.optimize();
        result.print("Final:\n");
        plotEvidenceMap2D(data, origin.x(), origin.y(), cell_size);
//    plt::savefig("1.png");
//    matplot::show();
        vector<double> x_plot, y_plot;
        for (int i=0; i<total_time_step + 1; i++){
            auto conf = result.at<Vector>(symbol('x', i));
            x_plot.push_back(conf.x());
            y_plot.push_back(conf.y());
        }
        matplot::plot(x_plot, y_plot,"--xr")->line_width(2);
    }
    matplot::save("1.png");

    return 0;
}