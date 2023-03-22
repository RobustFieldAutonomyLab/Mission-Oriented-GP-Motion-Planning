//
// Created by rfal on 12/7/22.
//
#pragma once


#ifndef GPMP_STR_SIGNEDDISTANCEFIELD_H
#define GPMP_STR_SIGNEDDISTANCEFIELD_H

#include "../third_party/edt/edt.hpp"
#include "../gpmp2/obstacle/SignedDistanceField.h"
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

inline std::vector<Matrix> signedDistanceField3D(const std::vector<Matrix> &ground_truth_map, float cell_size, float cell_size_z = 1){
    Matrix one_map;
    int z = ground_truth_map.size();
    int rows = ground_truth_map[0].rows();
    int cols = ground_truth_map[0].cols();

    one_map.setOnes(rows, cols);

    bool all_zero = true;
    bool cur_map[z*rows*cols];
    bool inv_map[z*rows*cols];
    for (int k = 0; k < z; k++){
        if (ground_truth_map[k].sum() != 0)
            all_zero = false;
        for (int i=0; i<rows; i++){
            for(int j=0; j<cols; j++){
                cur_map[k*rows*cols + i*cols+j] =  ground_truth_map[k](i,j);
                inv_map[k*rows*cols + i*cols+j] = !ground_truth_map[k](i,j);
            }
        }
    }

    std::vector<Matrix> field_3D;

    if(all_zero){
        Matrix this_layer;
        this_layer.setOnes(rows, cols);
        for (int i = 0; i < z; i++){
            field_3D.push_back(this_layer);
        }
        return field_3D;
    }

    float* inv_map_dist = edt::edt<bool>(cur_map,
                                cols, rows, z,
                                cell_size, cell_size, cell_size_z,true);
    float* map_dist = edt::edt<bool>(inv_map,
                                cols, rows, z,
                                cell_size, cell_size, cell_size_z,true);


    for (int k=0; k<z; k++){
        Matrix this_layer(rows, cols);
        for (int i=0; i<rows; i++){
            for(int j=0; j<cols; j++){
                int id = k*rows*cols + i*cols + j;
                double field_this = map_dist[id] - inv_map_dist[id];
//                cout<<" "<<inv_map_dist[id]<<" ";
                this_layer(i,j) = field_this;// * cell_size;
            }
        }
//        matplot::subplot(2, 1, 1);
//        plotEvidenceMap2D(this_layer, 0, 0, 1);
//        sleep(1);
        field_3D.push_back(this_layer);
    }

    return field_3D;
}

inline gpmp2::SignedDistanceField* buildSDF(double cell_size, double cell_size_z,
                    Point3 origin, Matrix &seafloor_map,
                    bool use_boundary, double sea_level = 0) {
    auto s_min = seafloor_map.minCoeff();
    auto s_max = seafloor_map.maxCoeff();
    int rows = seafloor_map.rows();
    int cols = seafloor_map.cols();

    double max_z = sea_level;
    double min_z = fmin(s_min, origin.z());

    int z_level = int( (max_z - min_z) / cell_size_z );

    static gpmp2::SignedDistanceField sdf(
            origin, cell_size, cell_size_z,
            rows, cols, z_level);

    std::vector<Matrix> data_3D;

    for (int z=0; z < z_level; z++){
        Matrix data;
        if(use_boundary && z * cell_size_z + origin.z() > sea_level){
            data.setOnes(seafloor_map.rows(), seafloor_map.cols());
        }
        else{
            data.setZero(seafloor_map.rows(), seafloor_map.cols());
            for (int i=0;i<rows;i++){
                for(int j=0;j<cols;j++){
                    if (seafloor_map(i,j) >= 0){
                        //mark occupied for all lands
                        data(i,j) = 1;
                    }
                    else if ( z * cell_size_z + origin.z() < seafloor_map(i,j)){
                        data(i,j) = 1;
                    }
                    else if (use_boundary){
                        if (i==0 || i==rows-1 || j==0 || j==cols-1) {
                            data(i, j) = 1;
                        }
                    }
//                cout<< z * cell_size_z + origin.z() <<endl;
                }
            }
        }
        data_3D.push_back(data);
    }

    for (int i=1;i<rows-1;i++){
        for(int j=1; j<cols-1; j++){
            std::vector<double> all_z;
            all_z.push_back(origin.z());
            for (int z=0; z < z_level-1; z++){
                if (data_3D[z](i,j) == 1){
                    all_z.push_back(origin.z() + cell_size_z * z);
                }
            }
            seafloor_map(i,j) = *max_element(all_z.begin(), all_z.end());
        }
    }

    std::vector<Matrix> fields = signedDistanceField3D(data_3D, cell_size, cell_size_z);
    int level = 0;
    for (auto field : fields){
//        plotEvidenceMap2D(field, 0, 0, 1);
//        sleep(1);
        sdf.initFieldData(level, field);
        level ++;
//        print(field);
    }
    return &sdf;

}

inline gpmp2::SignedDistanceField* loadSDF(
        double cell_size, double cell_size_z, Point3 origin, Matrix &seafloor_map, double sea_level,
        std::string sdf_path) {
    std::vector<Matrix> sdf_data = loadSDFData(sdf_path, cell_size, cell_size_z, origin.z(), sea_level);

    static gpmp2::SignedDistanceField sdf(
            origin, cell_size, cell_size_z,
            sdf_data);

    return &sdf;

}



#endif //GPMP_STR_SIGNEDDISTANCEFIELD_H
