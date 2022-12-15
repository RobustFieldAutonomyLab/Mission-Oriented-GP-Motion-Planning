#include <iostream>
#include "../include/Planning2D.h"

void planning2D(){
    double delta_t = 0.4;
    int total_time_step = 5;
    Pose2 start_pose = Pose2(5, 5, 0);
    Vector start_vel = Vector3(0.0, 0.0, 0.0);

    Pose2 end_pose = Pose2(45, 45, 0);
    Vector end_vel = Vector3(0.0, 0.0, 0.0);

    Vector avg_vel = Vector3((end_pose.x() - start_pose.x())/delta_t/total_time_step,
                             (end_pose.y() - start_pose.y())/delta_t/total_time_step,
                             (end_pose.theta() - start_pose.theta())/delta_t/total_time_step);
    vector <Pose2> ps;
    vector <Vector> vs;
    for (int i = 0; i<=5; i++){
        Pose2 pose = Pose2(start_pose.x() * (total_time_step-i)/total_time_step + end_pose.x() * i/total_time_step, start_pose.y() * (total_time_step-i)/total_time_step + end_pose.y() * i/total_time_step, start_pose.theta() * (total_time_step-i)/total_time_step + end_pose.theta() * i/total_time_step);
        ps.push_back(pose);
        vs.push_back(avg_vel);
    }
    Planning2D p2d;
    p2d.buildMap(1,Point2(0,0), 50,50);
    auto result = p2d.optimize(ps, vs, delta_t);
    for (auto pose : result){
        std::cout<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<std::endl;
    }
}

#include "../include/SignedDistanceField.h"

void test_edt(){
    Matrix input(6,5);
    input.setZero();
    input(0,3) = 1;
    input(1,1) = 1;input(1,2) = 1;input(1,3) = 1;
    input(2,1) = 1;input(2,2) = 1;input(2,3) = 1;
    input(3,1) = 1;input(3,2) = 1;input(3,3) = 1;
    input(4,1) = 1;input(4,2) = 1;input(4,3) = 1;
    Matrix output = distance_transform_edt(input);
    print(output);
    Matrix o = output.setOnes() - input;
    Matrix output2 = distance_transform_edt(o);
    print(output2);

}

void test_edt3D(){
    int z = 4;
    int rows = 5;
    int cols = 6;
    
    vector<Matrix> input3D;
    bool cur_map[4*5*6];
    bool inv_map[4*5*6];
    for (int i = 0; i<4; i++){
        Matrix input(5,6);
        input.setZero(5,6);
        if (i<3 && i>0){
            for (int j = 1; j < 4; j++){
                for (int k = 1; k < 5; k++){
                    input(j,k) = 1;
                }
            }
        }
        input3D.push_back(input);
        print(input);
    }
    int cnt = 0;
    for (auto layer:input3D){
        for (int i=0; i<rows; i++){
            for(int j=0; j<cols; j++){
                cur_map[cnt] =  layer(i,j);
                inv_map[cnt] = !layer(i,j);
                cnt++;
            }
        }
    }

    float* inv_map_dist = edt::edt<bool>(cur_map,
                                cols, rows, z,
                                1, 1, 1,false);
    float* map_dist = edt::edt<bool>(inv_map,
                                cols, rows, z,
                                1, 1, 1,false);
    
    for (int d = 0; d<4*5*6; d++){
        std::cout<< map_dist[d] <<" ";
    }                         

    for (int d = 0; d<4*5*6; d++){
        std::cout<< inv_map_dist[d] <<" ";
    }                         

    std::cout<<endl;
    vector<Matrix> field_3D;
    for (int k=0; k<4; k++){
        Matrix this_layer(rows, cols);
        for (int i=0; i<rows; i++){
            for(int j=0; j<cols; j++){
                int id = k*rows*cols + i*cols + j;
                std::cout<<id<<" ";
                double field_this = map_dist[id];// - inv_map_dist[id];
                this_layer(i,j) = field_this;
            }
            std::cout<<endl;
        }
        std::cout<<endl;
        field_3D.push_back(this_layer);
    }

    for (auto layer:field_3D){
        print(layer);
    }
    
}

int main() {
    planning2D();
//    test_edt3D();

    return 0;
}

