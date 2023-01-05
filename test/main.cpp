#include <iostream>
#include "../include/SignedDistanceField.h"
#include "../gpmp2/mission/Seafloor.h"
#include "../include/Visualization.h"
#include "../gpmp2/mission/WaterCurrentGrid.h"

using namespace std;
using namespace gtsam;

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

void test_getDistance(){
    Matrix sf = loadSeaFloorData("../data/depth_grid2.csv");
    plotEvidenceMap3D(sf, 0, 0, 1, 0);
    gpmp2::Seafloor sf_model(Point3(0,0,-4243), 1, sf);
    vector<Point3> target{Point3(40,20,-4200),
                          Point3(23.4, 32.1, -4194.8),
                          Point3(21.5, 11.5, -4125.5)};
    matplot::hold(matplot::on);
    for (auto t : target){
        double d = sf_model.getDistance(t);
        cout<< d <<endl;
        vector<double> X{t.x(), t.x()};
        vector<double> Y{t.y(), t.y()};
        vector<double> Z{t.z(), t.z() - d};
        auto l = matplot::plot3(X, Y, Z,"-ob")->line_width(2);
    }

    matplot::show();
}

void test_waterCurrentGrid(){
    vector<Matrix> current_grid_u = loadCurrentData("../data/u_mean.csv");
    vector<Matrix> current_grid_v = loadCurrentData("../data/v_mean.csv");
    vector<double> current_depth_grid = loadDepthData("../data/current_depth_map.csv");

    auto wcg = gpmp2::WaterCurrentGrid(Point3(0,0,-5000),
                                       1, 1000, 50, 60, 2,
                                       current_grid_u, current_grid_v);

    vector<Point3> target{Point3(40,20,-4200),
                          Point3(23.4, 32.1, -4194.8),
                          Point3(21.5, 11.5, -4125.5)};

    vector<Point3> lp{Point3(40,20,0),
                          Point3(23, 32, 0),
                          Point3(21, 11, 0)};

    vector<Point3> hp{Point3(41,21,1),
                      Point3(24, 33, 1),
                      Point3(22, 12, 1)};

    for (int i = 0; i<target.size(); i++){
        cout<< "lp u " << current_grid_u[lp[i].z()](lp[i].y(), lp[i].x())<<endl;
        cout<< "lp v " << current_grid_v[lp[i].z()](lp[i].y(), lp[i].x())<<endl;

        cout<< "hp u " << current_grid_u[hp[i].z()](hp[i].y(), hp[i].x())<<endl;
        cout<< "hp v " << current_grid_v[hp[i].z()](hp[i].y(), hp[i].x())<<endl;
        cout<<wcg.getVelocity(target[i])<<endl;
    }

}

int main() {
    test_waterCurrentGrid();

    return 0;
}

