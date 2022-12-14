//
// Created by rfal on 12/7/22.
//
#pragma once


#ifndef GPMP_STR_SIGNEDDISTANCEFIELD_H
#define GPMP_STR_SIGNEDDISTANCEFIELD_H

#include "../third_party/edt/edt.hpp"
#include <gtsam/base/Matrix.h>

#include "Visualization.h"

using namespace gtsam;

inline Matrix distance_transform_edt(Matrix input){
    int cols = input.cols();
    int rows = input.rows();
    bool labels[cols*rows];
    for (int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            labels[i*cols+j] = input(i,j);
        }
    }

    float* res = edt::edt<bool>(labels, cols, rows, 1, 1,true);
    Matrix output(rows, cols);
    for (int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            output(i,j) = res[i*cols+j];
        }
    }
    return output;

}

inline Matrix signedDistanceField2D(Matrix ground_truth_map, double cell_size){
    auto cur_map = ground_truth_map;
    Matrix one_map;
    one_map.setOnes(ground_truth_map.rows(), ground_truth_map.cols());
    if (ground_truth_map.sum() == 0 ){
        cur_map.setConstant(1000);
        return cur_map;
    }
    auto inv_map = one_map - cur_map;

    auto map_dist = distance_transform_edt(inv_map);

    auto inv_map_dist = distance_transform_edt(cur_map);

    auto field = map_dist - inv_map_dist;

    return field * cell_size;
}

inline vector<Matrix> signedDistanceField3D(vector<Matrix> ground_truth_map, double cell_size){
    Matrix one_map;

    int z = ground_truth_map.size();
    int rows = ground_truth_map[0].rows();
    int cols = ground_truth_map[0].cols();

    one_map.setOnes(rows, cols);

    int sum = 0;
    bool cur_map[z*rows*cols];
    bool inv_map[z*rows*cols];
    for (int k = 0; k < ground_truth_map.size(); k++){
        sum += ground_truth_map[k].sum();
        for (int i=0; i<rows; i++){
            for(int j=0; j<cols; j++){
                cur_map[k*rows*cols + i*cols+j] =  ground_truth_map[k](i,j);
                inv_map[k*rows*cols + i*cols+j] = !ground_truth_map[k](i,j);
            }
        }
    }

    vector<Matrix> field_3D;

    if(sum == 0){
        Matrix this_layer;
        this_layer.setOnes(rows, cols);
        for (int i = 0; i < z; i++){
            field_3D.push_back(this_layer);
        }
        return field_3D;
    }

    float* inv_map_dist = edt::edt<bool>(cur_map,
                                cols, rows, z,
                                1, 1, 1,true);
    float* map_dist = edt::edt<bool>(inv_map,
                                cols, rows, z,
                                1, 1, 1,true);


    for (int k=0; k<z; k++){
        Matrix this_layer(rows, cols);
        for (int i=0; i<rows; i++){
            for(int j=0; j<cols; j++){
                int id = k*rows*cols + i*cols + j;
                double field_this = map_dist[id] - inv_map_dist[id];
//                cout<<" "<<inv_map_dist[id]<<" ";
                this_layer(i,j) = field_this * cell_size;
            }
        }
//        matplot::subplot(2, 1, 1);
//        plotEvidenceMap2D(this_layer, 0, 0, 1);
//        sleep(1);
        field_3D.push_back(this_layer);
    }

    return field_3D;
}



#endif //GPMP_STR_SIGNEDDISTANCEFIELD_H
