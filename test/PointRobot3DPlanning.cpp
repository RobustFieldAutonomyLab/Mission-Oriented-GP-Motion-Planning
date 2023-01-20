
//
// Created by rfal on 12/8/22.
//

#include "Planning3D.h"
//#include "Visualization.h"

void planning3D(Matrix seafloor_map){
    double delta_t = 0.4;
    int total_time_step = 100;
    Point3 start_pose = Point3(50, 50, -375);
    Vector start_vel = Vector3(0.0, 0.0, 0.0);

    Point3 end_pose = Point3(450, 450, -258);
    Vector end_vel = Vector3(0.0, 0.0, 0.0);

    Vector avg_vel = Vector3((end_pose.x() - start_pose.x())/delta_t/total_time_step,
                             (end_pose.y() - start_pose.y())/delta_t/total_time_step,
                             (end_pose.z() - start_pose.z())/delta_t/total_time_step);
    vector <Vector> ps;
    vector <Vector> vs;
    for (int i = 0; i<=total_time_step; i++){
        Vector pose = Vector3(start_pose.x() * (total_time_step-i)/total_time_step + end_pose.x() * i/total_time_step,
                              start_pose.y() * (total_time_step-i)/total_time_step + end_pose.y() * i/total_time_step,
                              start_pose.z() * (total_time_step-i)/total_time_step + end_pose.z() * i/total_time_step);
        ps.push_back(pose);
        vs.push_back(avg_vel);
    }
    Planning3D p3d(0.2, 0.2, 1,5);
    p3d.buildMap(seafloor_map, Point3(0,0,-425), 1, 10);
    auto result = p3d.optimize(ps, vs, delta_t);
    vector<double> X, Y, Z;
    for (auto pose : result){
        X.push_back(pose[0]);
        Y.push_back(pose[1]);
        Z.push_back(pose[2]);
    }

    auto l = matplot::plot3(X, Y, Z,"-ob");
    matplot::show();

}

void simplier(Matrix seafloor_map){
    double delta_t = 0.4;
    int total_time_step = 100;
    Point3 start_pose = Point3(5, 5, -4220);
    Vector start_vel = Vector3(0.0, 0.0, 0.0);

    Point3 end_pose = Point3(45, 45, -4185);
    Vector end_vel = Vector3(0.0, 0.0, 0.0);

    Vector avg_vel = Vector3((end_pose.x() - start_pose.x())/delta_t/total_time_step,
                             (end_pose.y() - start_pose.y())/delta_t/total_time_step,
                             (end_pose.z() - start_pose.z())/delta_t/total_time_step);
    vector <Vector> ps;
    vector <Vector> vs;
    for (int i = 0; i<=total_time_step; i++){
        Vector pose = Vector3(start_pose.x() * (total_time_step-i)/total_time_step + end_pose.x() * i/total_time_step,
                              start_pose.y() * (total_time_step-i)/total_time_step + end_pose.y() * i/total_time_step,
                              start_pose.z() * (total_time_step-i)/total_time_step + end_pose.z() * i/total_time_step);
        ps.push_back(pose);
        vs.push_back(avg_vel);
    }
    Planning3D p3d(5, 0.2, 0.2,5);
    p3d.buildMap(seafloor_map,Point3(0,0,-4243),1,1);
    auto result = p3d.optimize(ps, vs, delta_t);
    vector<double> X, Y, Z;
    for (auto pose : result){
        X.push_back(pose[0]);
        Y.push_back(pose[1]);
        Z.push_back(pose[2]);
    }
    plotEvidenceMap3D(seafloor_map,0,0,1,MESH);
    matplot::hold(matplot::on);
    auto l = matplot::plot3(X, Y, Z,"-ob");
    matplot::show();
}

int main(){
//    Matrix data = loadSeaFloorData("../data/depth_grid1.csv");
//    plotEvidenceMap3D(data,0,0,1,1);
//    matplot::show();
//    planning3D(data);
////    matplot::save("1.png");
    Matrix data = loadSeaFloorData("../data/depth_grid2.csv");
//    matplot::subplot(2, 1, 0);
//    plotEvidenceMap3D(data,0,0,1,1);
//    matplot::hold(true);
//    vector<double> X, Y, Z;
//    X.push_back(5);
//    Y.push_back(5);
//    Z.push_back(-423);
//    X.push_back(38);
//    Y.push_back(38);
//    Z.push_back(-418);
//
//    auto l = matplot::plot3(X, Y, Z,"-ob");

//    matplot::show();
    simplier(data);



}