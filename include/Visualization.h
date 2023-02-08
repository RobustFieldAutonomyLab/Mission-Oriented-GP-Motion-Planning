//
// Created by rfal on 12/7/22.
//
#pragma once


#ifndef GPMP_STR_VISUALIZATION_H
#define GPMP_STR_VISUALIZATION_H

#include <matplot/matplot.h>
#include <gtsam/geometry/Point2.h>
#include <fstream>

enum PLOT_TYPE{MESH, POINT, DOWNSIZE_MESH};


inline gtsam::Point2 get_center(double x, double y, double origin_x, double origin_y, double cell_size ){
    gtsam::Point2 center = gtsam::Point2(int( (y-origin_y)/cell_size ),
                           int( (x-origin_x)/cell_size ) );
    return center;
}

inline gtsam::Point2 get_dim(double h, double w, double cell_size ){
    gtsam::Point2 center = gtsam::Point2(int(w/cell_size), int(h/cell_size));
    return center;
}
inline void plotEvidenceMap2D(gtsam::Matrix prob_grid, double origin_x, double origin_y, double cell_size){
    using namespace matplot;
    int grid_rows = prob_grid.rows();
    int grid_cols = prob_grid.cols();
    double grid_corner_x = origin_x + (grid_cols - 1) * cell_size;
    double grid_corner_y = origin_y + (grid_rows - 1) * cell_size;
    std::vector<std::vector<double>> C;
    for (int i = 0; i < grid_rows; i++){
        std::vector<double> C_tmp;
        for (int j = 0; j < grid_cols; j++){
            C_tmp.push_back(prob_grid(i,j));
        }
        C.push_back(C_tmp);
    }

    auto im = image(origin_x, grid_corner_x, origin_y, grid_corner_y,C);
    hold(on);
}

inline void plotEvidenceMap3D(gtsam::Matrix prob_grid,
                       double origin_x,
                       double origin_y,
                       double cell_size,
                       PLOT_TYPE type){
    using namespace matplot;
    int grid_rows = prob_grid.rows();
    int grid_cols = prob_grid.cols();
    double grid_corner_x = origin_x + (grid_cols - 1) * cell_size;
    double grid_corner_y = origin_y + (grid_rows - 1) * cell_size;
    std::vector<std::vector<double>> Z;

    int cut_x = 0;
    if (type == MESH){
        auto [X, Y] =
                meshgrid(iota(origin_x+cut_x, cell_size, grid_corner_x),
                               iota(origin_y, cell_size, grid_corner_y));
        for (int i=0; i < grid_rows; i++){
            std::vector<double> this_line_Z;
            for (int j = cut_x; j < grid_cols; j++){
                this_line_Z.push_back(prob_grid(i,j));
            }
            Z.push_back(this_line_Z);
        }
        mesh(X, Y, Z);
    }
    else if (type == DOWNSIZE_MESH){
        int grid_size = std::max(grid_cols, grid_rows);
        double scale = double(grid_size) / 100;
        auto [X, Y] =
                meshgrid(iota(origin_x, cell_size * scale, grid_corner_x),
                               iota(origin_y, cell_size * scale, grid_corner_y));
        for (int i=0; i < grid_rows; i+=scale){
            std::vector<double> this_line_Z;
            for (int j = 0; j < grid_cols; j+=scale){
                this_line_Z.push_back(prob_grid(i,j));
            }
            Z.push_back(this_line_Z);
        }
        mesh(X, Y, Z);
    }
    else if (type == POINT){
        std::vector<double> X, Y, Z, sizes;

        for (int i=0; i < grid_rows; i++){
            for (int j = 0; j < grid_cols; j++){
                X.push_back(j);
                Y.push_back(i);
                Z.push_back(prob_grid(i,j));
                sizes.push_back(1);
            }
        }
        scatter3(X, Y, Z, sizes);
    }

    hold(on);
}

inline gtsam::Matrix loadSeaFloorData(std::string file_name){
    std::fstream depth_file;
    depth_file.open(file_name, std::ios::in);
    double x_width_d, y_width_d;
    int x_width, y_width;
    depth_file>>x_width_d;
    depth_file>>y_width_d;
    x_width = x_width_d;
    y_width = y_width_d;
    gtsam::Matrix data(x_width, y_width);
    double tmp;
    for (int i=0; i<x_width; i++){
        for (int j=0; j<y_width; j++){
            depth_file>>tmp;
            data(i,j) = tmp ;
        }
    }
    return data;

}

inline void savePath(std::vector<double> X, std::vector<double> Y, std::vector<double> Z, string filename){
    std::fstream file;
    file.open(filename, std::ios::out);
    for (int i = 0; i < X.size(); i++){
        file << X[i]<<", "<<Y[i]<<", "<<Z[i]<<endl;
    }
    file.close();
}

inline std::vector<gtsam::Matrix> loadCurrentData(std::string file_name){
    std::fstream depth_file;
    depth_file.open(file_name, std::ios::in);
    int x_width, y_width, z_width;
    depth_file>>x_width;
    depth_file>>y_width;
    depth_file>>z_width;
    std::vector<gtsam::Matrix> data;
    for (int i = 0; i<z_width; i++){
        gtsam::Matrix this_layer(x_width, y_width);
        double tmp;
        for (int j=0; j<x_width; j++){
            for (int k=0; k<y_width; k++){
                depth_file>>tmp;
                this_layer(j,k) = tmp;
            }
        }
        data.push_back(this_layer);
    }

    return data;
}

inline std::vector<double> loadDepthData(std::string file_name){
    std::fstream depth_file;
    depth_file.open(file_name, std::ios::in);
    int x_width;
    depth_file>>x_width;
    std::vector<double> data;
    for (int i = 0; i<x_width; i++){
        double tmp;
        depth_file>>tmp;
        data.push_back(tmp);
    }
    return data;
}

inline std::vector<gtsam::Matrix> loadSDFData(std::string file_name,
                                  double cell_size, double cell_size_z, double origin_z, double sea_level){
    std::fstream sdf_file;
    sdf_file.open(file_name, std::ios::in);
    int x_width, y_width, z_width;
    double x_width_d, y_width_d, z_width_d;
    double r_cell_size, r_cell_size_z, r_origin_z, r_sea_level;
    sdf_file>>z_width_d>>x_width_d>>y_width_d;
    x_width = int(x_width_d); y_width = int(y_width_d); z_width = int(z_width_d);
    sdf_file>>r_cell_size>>r_cell_size_z>>r_origin_z>>r_sea_level;
    if (r_cell_size != cell_size ||
            r_cell_size_z != cell_size_z ||
                r_origin_z != origin_z ||
                    r_sea_level != sea_level){
        std::cout<<"Different cell size or origin for SDF data and motion planning!"<<std::endl;
        return std::vector<gtsam::Matrix>{};
    }
    std::vector<gtsam::Matrix> data;
    for (int i = 0; i<z_width; i++){
        gtsam::Matrix this_layer(x_width, y_width);
        double tmp;
        for (int j=0; j<x_width; j++){
            for (int k=0; k<y_width; k++){
                sdf_file>>tmp;
                this_layer(j,k) = tmp;
            }
        }
        data.push_back(this_layer);
    }
    return data;
}


#endif //GPMP_STR_VISUALIZATION_H
