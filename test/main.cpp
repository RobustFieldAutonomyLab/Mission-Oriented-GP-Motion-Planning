#include <iostream>
#include "../include/SignedDistanceField.h"

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

int main() {
    test_edt3D();

    return 0;
}

