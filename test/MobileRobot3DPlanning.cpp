#include "Planning3DUUV.h"
//#include "matplot/matplot.h"

using namespace std;

void simplier(Matrix seafloor_map){
    double delta_t = 0.4;
    int total_time_step = 100;
    Pose3 start_pose = Pose3(Rot3(),Point3(5, 5, -4220));
    Vector start_vel;
    Vector6 start_vel6;
    start_vel6 << Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0);
    start_vel = start_vel6;

    Pose3 end_pose = Pose3(Rot3(), Point3(45, 45, -4185));
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
    Planning3DUUV p3d(true, 3, 0.1, 0.2,0.2);
    p3d.buildMap(1,1,Point3(0,0,-4243),seafloor_map);
    auto result = p3d.optimize(ps, vs, delta_t);
    vector<double> X, Y, Z;
    for (auto pose : result){
        X.push_back(pose.x());
        Y.push_back(pose.y());
        Z.push_back(pose.z());
    }
    plotEvidenceMap3D(seafloor_map,0,0,1,0);
    matplot::hold(matplot::on);
    auto l = matplot::plot3(X, Y, Z,"-ob");
    matplot::show();
}

int main(){

    Matrix data = loadSeaFloorData("../data/depth_grid2.csv");
    simplier(data);
}