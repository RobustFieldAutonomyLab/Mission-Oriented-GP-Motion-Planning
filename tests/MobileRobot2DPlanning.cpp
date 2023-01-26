//
// Created by rfal on 12/1/22.
//
#include <iostream>
#include <cmath>
#include <math.h>
#include <string>
#include <vector>

#include "Planning2D.h"


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

// settings
    double total_time_sec = 10.0;
    int total_time_step = 20;
    double total_check_step = 50.0;
    double delta_t = total_time_sec / total_time_step;
    int check_inter = int(total_check_step / total_time_step - 1);

    Pose2 start_conf = Pose2(0, 0, 0);
    Vector start_vel = Vector3(0, 0, 0);
    Pose2 end_conf = Pose2(17, 14, 0);
    Vector end_vel = Vector3(0, 0, 0);
    Vector avg_vel = (Vector3(end_conf.x()-start_conf.x(),
                                end_conf.y()-start_conf.y(),
                                    end_conf.theta()-end_conf.theta())
            / total_time_step) / delta_t;

// plot param
    double pause_time = total_time_sec / total_time_step;

    vector <Pose2> ps;
    vector <Vector> vs;

    for (int i = 0; i < total_time_step + 1; i++) {
        Pose2 pose = Pose2(start_conf.x() * (total_time_step-i)/total_time_step +
                end_conf.x() * i/total_time_step,
                start_conf.y() * (total_time_step-i)/total_time_step +
                end_conf.y() * i/total_time_step,
                start_conf.theta() * (total_time_step-i)/total_time_step +
                end_conf.theta() * i/total_time_step);
        ps.push_back(pose);
        vs.push_back(avg_vel);
    }

    Planning2D p2d;
    p2d.buildMap(cell_size, origin, data);
    vector<double> x_plot, y_plot;
    auto result = p2d.optimize(ps, vs, delta_t);
    for (auto pose : result){
        x_plot.push_back(pose.x());
        y_plot.push_back(pose.y());

    }

    plotEvidenceMap2D(data, origin.x(), origin.y(), cell_size);

    matplot::plot(x_plot, y_plot,"--xr")->line_width(2);

    matplot::save("1.png");

    return 0;
}