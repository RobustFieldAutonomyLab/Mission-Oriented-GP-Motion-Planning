//
// Created by rfal on 12/7/22.
//
#pragma once


#ifndef GPMP_STR_VISUALIZATION_H
#define GPMP_STR_VISUALIZATION_H

#include <matplot/matplot.h>
#include <gtsam/geometry/Point2.h>
#include <fstream>

inline Point2 get_center(double x, double y, double origin_x, double origin_y, double cell_size ){
    Point2 center = Point2(int( (y-origin_y)/cell_size ),
                           int( (x-origin_x)/cell_size ) );
    return center;
}

inline Point2 get_dim(double h, double w, double cell_size ){
    Point2 center = Point2(int(w/cell_size), int(h/cell_size));
    return center;
}
inline void plotEvidenceMap2D(Matrix prob_grid, double origin_x, double origin_y, double cell_size){
    using namespace matplot;
    int grid_rows = prob_grid.rows();
    int grid_cols = prob_grid.cols();
    double grid_corner_x = origin_x + (grid_cols - 1) * cell_size;
    double grid_corner_y = origin_y + (grid_rows - 1) * cell_size;
    vector<vector<double>> C;
    for (int i = 0; i < grid_rows; i++){
        vector<double> C_tmp;
        for (int j = 0; j < grid_cols; j++){
            C_tmp.push_back(prob_grid(i,j));
        }
        C.push_back(C_tmp);
    }

    image(origin_x, grid_corner_x, origin_y, grid_corner_y,C);
    hold(on);
}

inline void plotEvidenceMap3D(Matrix prob_grid,
                       double origin_x,
                       double origin_y,
                       double cell_size,
                       int type){
    using namespace matplot;
    int grid_rows = prob_grid.rows();
    int grid_cols = prob_grid.cols();
    double grid_corner_x = origin_x + (grid_cols - 1) * cell_size;
    double grid_corner_y = origin_y + (grid_rows - 1) * cell_size;
    vector<vector<double>> Z;

    if (type == 0){
        auto [X, Y] = meshgrid(iota(origin_x, cell_size, grid_corner_x),
                               iota(origin_y, cell_size, grid_corner_y));
        for (int i=0; i < grid_rows; i++){
            vector<double> this_line_Z;
            for (int j = 0; j < grid_cols; j++){
                this_line_Z.push_back(prob_grid(i,j));
            }
            Z.push_back(this_line_Z);
        }
        mesh(X, Y, Z);
    }
    else if (type == 1){
        vector<double> X, Y, Z;

        for (int i=0; i < grid_rows; i++){
            for (int j = 0; j < grid_cols; j++){
                X.push_back(i);
                Y.push_back(j);
                Z.push_back(prob_grid(i,j));
            }
        }
        scatter3(X, Y, Z);
    }

    hold(on);
}

inline Matrix loadSeaFloorData(string file_name){
    fstream depth_file;
    depth_file.open(file_name, ios::in);
    int x_width, y_width;
    depth_file>>x_width;
    depth_file>>y_width;
    Matrix data(x_width, y_width);
    double tmp;
    for (int i=0; i<x_width; i++){
        for (int j=0; j<y_width; j++){
            depth_file>>tmp;
            data(i,j) = tmp ;
        }
    }
    return data;

}


#endif //GPMP_STR_VISUALIZATION_H
