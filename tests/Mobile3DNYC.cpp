#include "Planning3DUUV.h"
//#include "matplot/matplot.h"

using namespace std;

void run_small(Matrix seafloor_map){
    double delta_t = 0.4;
    int total_time_step = 300;
    Pose3 start_pose = Pose3(Rot3(),Point3(40,10,  -15));
    Vector start_vel;
    Vector6 start_vel6;
    start_vel6 << Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0);
    start_vel = start_vel6;

    Pose3 end_pose = Pose3(Rot3(), Point3(50, 100, -10));
    Vector end_vel;
    Vector6 end_vel6;
    end_vel6 << Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0);
    end_vel = end_vel6;

    Vector avg_vel;
    Vector6 avg_vel6;
    avg_vel6 << Vector3((end_pose.rotation().roll() - start_pose.rotation().roll())/delta_t/total_time_step,
                        (end_pose.rotation().pitch() - start_pose.rotation().pitch())/delta_t/total_time_step,
                        (end_pose.rotation().yaw() - start_pose.rotation().yaw())/delta_t/total_time_step),
            Vector3((end_pose.x() - start_pose.x())/delta_t/total_time_step,
                    (end_pose.y() - start_pose.y())/delta_t/total_time_step,
                    (end_pose.z() - start_pose.z())/delta_t/total_time_step);
    avg_vel = avg_vel6;
    vector <Pose3> ps;
    vector <Vector> vs;
    for (int i = 0; i<=total_time_step; i++){
        Pose3 pose = Pose3(Rot3(),
                           Point3(start_pose.x() * (total_time_step-i)/total_time_step + end_pose.x() * i/total_time_step,
                                  start_pose.y() * (total_time_step-i)/total_time_step + end_pose.y() * i/total_time_step,
                                  start_pose.z() * (total_time_step-i)/total_time_step + end_pose.z() * i/total_time_step));
        ps.push_back(pose);
        if (i == 0)
            vs.push_back(start_vel);
        else if(i == total_time_step)
            vs.push_back(end_vel);
        else
            vs.push_back(avg_vel);
    }
    Planning3DUUVParameter param;
    param.use_vehicle_dynamics = true;
    param.dynamics_sigma = 1;

    param.obstacle_epsilon_dist = 1;
    param.obstacle_cost_sigma = 1;

    param.vehicle_size = 1;

    param.seafloor_mission = true;
    param.seafloor_cost_sigma = 1;
    param.seafloor_dist = 1.2;

    param.check_inter = 10;//

    param.use_current = false;

    Planning3DUUV p3d(param);
    seafloor_map = p3d.buildMap(1,1,Point3(0,0,-35),seafloor_map, -6);
    auto result = p3d.optimize(ps, vs, delta_t);
    vector<double> X, Y, Z;
    for (auto pose : result){
        X.push_back(pose.x());
        Y.push_back(pose.y());
        Z.push_back(pose.z());
    }
    plotEvidenceMap3D(seafloor_map,0,0,1,MESH);
    matplot::hold(matplot::on);
//    plotEvidenceMap3D(seafloor_map,0,0,1,POINT);
    auto l = matplot::plot3(X, Y, Z,"-ob");
//    vector<double> sx, sy, sz;
//    auto l2 = matplot::plot3(sx, sy, sz, "r*");
    matplot::show();
    savePath(X, Y, Z, "../../data/NYC1.txt");
}

void run_big(Matrix seafloor_map, string sdf_path){
    double delta_t = 0.4;
    int total_time_step = 300;
    Pose3 start_pose = Pose3(Rot3(),Point3(400,100, -15));
    Vector start_vel;
    Vector6 start_vel6;
    start_vel6 << Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0);
    start_vel = start_vel6;

    Pose3 end_pose = Pose3(Rot3(), Point3(500, 1000, -10));
    Vector end_vel;
    Vector6 end_vel6;
    end_vel6 << Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0);
    end_vel = end_vel6;

    Vector avg_vel;
    Vector6 avg_vel6;
    avg_vel6 << Vector3((end_pose.rotation().roll() - start_pose.rotation().roll())/delta_t/total_time_step,
                        (end_pose.rotation().pitch() - start_pose.rotation().pitch())/delta_t/total_time_step,
                        (end_pose.rotation().yaw() - start_pose.rotation().yaw())/delta_t/total_time_step),
            Vector3((end_pose.x() - start_pose.x())/delta_t/total_time_step,
                    (end_pose.y() - start_pose.y())/delta_t/total_time_step,
                    (end_pose.z() - start_pose.z())/delta_t/total_time_step);
    avg_vel = avg_vel6;
    vector <Pose3> ps;
    vector <Vector> vs;
    for (int i = 0; i<=total_time_step; i++){
        Pose3 pose = Pose3(Rot3(),
                           Point3(start_pose.x() * (total_time_step-i)/total_time_step + end_pose.x() * i/total_time_step,
                                  start_pose.y() * (total_time_step-i)/total_time_step + end_pose.y() * i/total_time_step,
                                  start_pose.z() * (total_time_step-i)/total_time_step + end_pose.z() * i/total_time_step));
        ps.push_back(pose);
        if (i == 0)
            vs.push_back(start_vel);
        else if(i == total_time_step)
            vs.push_back(end_vel);
        else
            vs.push_back(avg_vel);
    }
    Planning3DUUVParameter param;
    param.use_vehicle_dynamics = true;
    param.dynamics_sigma = 1;

    param.obstacle_epsilon_dist = 1;
    param.obstacle_cost_sigma = 1;

    param.vehicle_size = 1;

    param.seafloor_mission = true;
    param.seafloor_cost_sigma = 1;
    param.seafloor_dist = 1.2;

    param.check_inter = 10;//

    param.use_current = false;

    Planning3DUUV p3d(param);
    double cell_size = 1;
    double cell_size_z = 1;
    //TODO: edit here
    seafloor_map = p3d.buildMap(cell_size,cell_size_z,Point3(0,0,-35),seafloor_map, -6, sdf_path);
    auto result = p3d.optimize(ps, vs, delta_t);
    vector<double> X, Y, Z, X_floor, Z_floor;
    for (auto pose : result){
        X.push_back(pose.x());
        Y.push_back(pose.y());
        Z.push_back(pose.z());
    }
    plotEvidenceMap3D(seafloor_map,0,0,cell_size,DOWNSIZE_MESH);
    matplot::hold(matplot::on);
    auto l = matplot::plot3(X, Y, Z,"-ob");
//    vector<double> sx, sy, sz;
//    auto l2 = matplot::plot3(sx, sy, sz, "r*");
    matplot::show();
//    draw(X, Y, Z, seafloor_map);
}

int main(){
    Matrix data = loadSeaFloorData("../../data/NYC/depth_grid_NYC_small.csv");
//    Matrix data = loadSeaFloorData("../../data/NYC/depth_grid_NYC.csv");
//    run_big(data, "../../data/NYC/sdfNYC10.txt");
    run_small(data);
}