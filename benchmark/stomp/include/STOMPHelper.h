#pragma once

#include "stomp/stomp.h"
#include "gtsam/geometry/Point3.h"
#include "../include/SignedDistanceField.h"
#include "../gpmp2/mission/Seafloor.h"
using Trajectory = Eigen::MatrixXd;

struct STOMPParameter{
    gtsam::Point3 origin;
    double cell_size;
    double cell_size_z;
    double sea_level;

    double vehicle_size;
    double dist_sdf;
    double w_sdf;

    double delta_t;
    int total_time_steps;

    Vector3 std_dev;
};

class STOMPHelper : public stomp::Task
{
public:

    STOMPHelper(const std::string& file_name,
                const STOMPParameter& params);

    bool generateNoisyParameters(const Eigen::MatrixXd& parameters,
                                                      std::size_t start_timestep,
                                                      std::size_t num_timesteps,
                                                      int iteration_number,
                                                      int rollout_number,
                                                      Eigen::MatrixXd& parameters_noise,
                                                      Eigen::MatrixXd& noise) override;

    bool computeCosts(const Eigen::MatrixXd& parameters,
                      std::size_t start_timestep,
                      std::size_t num_timesteps,
                      int iteration_number,
                      Eigen::VectorXd& costs,
                      bool& validity) override;

    bool computeNoisyCosts(const Eigen::MatrixXd& parameters,
                           std::size_t start_timestep,
                           std::size_t num_timesteps,
                           int iteration_number,
                           int rollout_number,
                           Eigen::VectorXd& costs,
                           bool& validity) override;

    bool filterParameterUpdates(std::size_t start_timestep,
                                std::size_t num_timesteps,
                                int iteration_number,
                                const Eigen::MatrixXd& parameters,
                                Eigen::MatrixXd& updates) override;

protected:
    bool smoothParameterUpdates(std::size_t start_timestep,
                                std::size_t num_timesteps,
                                int iteration_number,
                                Eigen::MatrixXd& updates);

protected:
    std::vector<double> std_dev_;
    Eigen::MatrixXd smoothing_M_;

    gtsam::Point3 origin_, corner_;

    double cell_size_;
    double vehicle_size_;

    gpmp2::Seafloor* sf_;
    double dist_sf_;

    gpmp2::SignedDistanceField* sdf_;
    double dist_sdf_;


};